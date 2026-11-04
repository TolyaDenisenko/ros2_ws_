// ============================================================
// ESP32-S3: гироскутер + лидар пылесоса + MPU6050 ↔ ROS 2
//
// Что делает:
//   - принимает команды скоростей колёс от ПК и передаёт их плате
//     гироскутера (EFeru hoverboard-firmware-hack-FOC) по UART;
//   - читает телеметрию гироскутера (об/мин колёс), считает одометрию
//     и шлёт её на ПК 50 раз в секунду;
//   - читает лидар (отдельная задача FreeRTOS) и шлёт скан кусками
//     по мере вращения;
//   - отвечает на PING (синхронизация часов ПК и ESP32).
// Связь с ПК — Wi-Fi/UDP или USB, выбирается при сборке (transport.h).
// Формат пакетов — protocol.h.
// ============================================================
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stdarg.h>

#include "config.h"
#include "protocol.h"
#include "transport.h"

static Transport pc;   // канал связи с ПК (Wi-Fi или USB)

/* ---------- Отправка пакетов ---------- */
static uint16_t txSeq = 0;
static portMUX_TYPE seqMux = portMUX_INITIALIZER_UNLOCKED;

static inline uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }

// Собирает и отправляет пакет. Потокобезопасно (loop и задача лидара).
static void sendPacket(uint8_t type, uint64_t stamp, const void * payload, size_t len)
{
  uint8_t b[proto::MAX_PACKET];   // на стеке вызывающей задачи — без гонок между задачами
  if (proto::HEADER_SIZE + len + proto::CRC_SIZE > proto::MAX_PACKET) return;
  portENTER_CRITICAL(&seqMux);
  const uint16_t seq = txSeq++;
  portEXIT_CRITICAL(&seqMux);
  const size_t n = proto::build(b, type, seq, stamp, (const uint8_t *)payload, len);
  pc.send(b, n);
}

// Отладочное сообщение на ПК (нода печатает его в свой лог).
static void logf(const char * fmt, ...)
{
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  if (n > 0) sendPacket(proto::LOG, nowUs(), msg, min(n, (int)sizeof(msg) - 1));
}

/* ---------- Состояние движения ---------- */
static float odomX = 0, odomY = 0, odomTh = 0;   // м, м, рад
static float speedL = 0, speedR = 0;              // измеренные, мм/с
static float tgtL = 0, tgtR = 0;                  // целевые, мм/с
static uint32_t lastCmdMs = 0;

/* ---------- IMU: удержание курса при езде прямо ---------- */
static MPU6050 mpu;
static bool mpuOk = false;
static bool straightMode = false;
static float straightTargetYaw = 0;

static bool initImu()
{
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  mpu.initialize();
  if (!mpu.testConnection()) return false;
  mpu.CalibrateGyro(6);       // робот должен стоять неподвижно
  mpu.CalibrateAccel(6);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  return true;
}

// Курс по гироскопу (простое интегрирование, дрейфует — годится только
// для коротких отрезков «езды прямо»). 0, если IMU нет.
static float imuYaw()
{
  static float yaw = 0;
  static uint32_t lastUs = micros();
  if (!mpuOk) return 0;
  const uint32_t t = micros();
  const float dt = (t - lastUs) * 1e-6f;
  lastUs = t;
  const float gz = mpu.getRotationZ() / 3754.5f;   // ±500 °/с: 65.5 LSB/(°/с) → рад/с
  yaw += gz * dt;
  yaw = atan2f(sinf(yaw), cosf(yaw));
  return yaw;
}

/* ---------- Гироскутер: команды ---------- */
// Кадр SerialCommand прошивки FOC: 0xABCD, steer, speed, XOR-сумма (int16 LE)
static void hbSend(int16_t steer, int16_t speed)
{
  uint8_t buf[8];
  const uint16_t start = 0xABCD;
  const uint16_t cs = start ^ (uint16_t)steer ^ (uint16_t)speed;
  memcpy(buf, &start, 2);
  memcpy(buf + 2, &steer, 2);
  memcpy(buf + 4, &speed, 2);
  memcpy(buf + 6, &cs, 2);
  HB_SERIAL.write(buf, sizeof(buf));
}

// Целевые скорости колёс → steer/speed гироскутера (+ коррекция курса по IMU).
// Коэффициенты 400/600 подобраны вручную под режим VLT_MODE прошивки FOC.
static void hbUpdate()
{
  const float vl = tgtL / 1000.0f, vr = tgtR / 1000.0f;
  const float lin = (vl + vr) * 0.5f;
  float ang = (vl - vr) / WHEEL_BASE;
  if (straightMode && fabsf(lin) > 0.05f) {
    const float err = atan2f(sinf(straightTargetYaw - imuYaw()), cosf(straightTargetYaw - imuYaw()));
    ang += constrain(err * YAW_KP, -0.5f, 0.5f);
  }
  hbSend((int16_t)constrain(ang * 400.0f, -1000.0f, 1000.0f),
         (int16_t)constrain(lin * 600.0f, -1000.0f, 1000.0f));
}

/* ---------- Гироскутер: телеметрия и одометрия ---------- */
// Кадр SerialFeedback прошивки FOC (Src/main.c), 9 × uint16 LE:
//   start(0xABCD) cmd1 cmd2 speedR_meas speedL_meas batVoltage boardTemp cmdLed checksum
// Разбор побайтный с поиском маркера — после потери байта поток не сбивается.
static void hbReadFeedback()
{
  static uint8_t fb[18];
  static uint8_t len = 0;
  static uint32_t lastMs = millis();
  while (HB_SERIAL.available()) {
    const uint8_t b = HB_SERIAL.read();
    if (len == 0 && b != 0xCD) continue;
    if (len == 1 && b != 0xAB) { len = (b == 0xCD); continue; }
    fb[len++] = b;
    if (len < sizeof(fb)) continue;
    len = 0;

    uint16_t w[9];
    memcpy(w, fb, sizeof(fb));
    if ((w[0] ^ w[1] ^ w[2] ^ w[3] ^ w[4] ^ w[5] ^ w[6] ^ w[7]) != w[8]) continue;

    const float vr = (int16_t)w[3] * RPM_TO_MPS;     // м/с
    const float vl = -(int16_t)w[4] * RPM_TO_MPS;    // левый мотор смонтирован зеркально
    speedL = vl * 1000.0f;
    speedR = vr * 1000.0f;

    const uint32_t now = millis();
    const float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    const float v = (vl + vr) * 0.5f;
    const float w_z = (vr - vl) / WHEEL_BASE;
    odomX += v * cosf(odomTh) * dt;
    odomY += v * sinf(odomTh) * dt;
    odomTh = atan2f(sinf(odomTh + w_z * dt), cosf(odomTh + w_z * dt));
  }
}

static void sendOdom()
{
  const float v[5] = {odomX, odomY, odomTh, speedL, speedR};
  sendPacket(proto::ODOM, nowUs(), v, sizeof(v));
}

/* ---------- Команды от ПК ---------- */
static void handlePacket(const uint8_t * pkt, size_t n)
{
  uint8_t type;
  uint64_t stamp;
  const uint8_t * p;
  size_t len;
  if (!proto::parse(pkt, n, type, stamp, p, len)) return;
  pc.accept_sender();

  switch (type) {
    case proto::CMD: {
      if (len != 4) return;
      int16_t l, r;
      memcpy(&l, p, 2);
      memcpy(&r, p + 2, 2);
      tgtL = l;
      tgtR = r;
      lastCmdMs = millis();
      // Колёса крутятся почти одинаково — едем прямо, держим курс по IMU
      const bool straight = fabsf(fabsf(tgtL) - fabsf(tgtR)) < 10.0f && fabsf(tgtL) > 10.0f;
      if (straight && !straightMode) straightTargetYaw = imuYaw();
      straightMode = straight;
      break;
    }
    case proto::PING:
      sendPacket(proto::PONG, nowUs(), &stamp, sizeof(stamp));
      break;
    case proto::RESET_ODOM:
      odomX = odomY = odomTh = 0;
      logf("odometry reset");
      break;
  }
}

/* ---------- Лидар ---------- */
// Пакет лидара: заголовок 55 AA 03 08, затем 32 байта тела:
//   скорость u16, угол начала u16, 8 × (дистанция u16 + качество u8), угол конца u16
static constexpr uint8_t LIDAR_HDR[4] = {0x55, 0xAA, 0x03, 0x08};
static constexpr size_t LIDAR_BODY = 32;
static constexpr uint8_t INTENSITY_MIN = 2;      // ниже — измерение недостоверно
static constexpr float MAX_SPREAD_DEG = 20.0f;   // больше — пакет битый
static constexpr size_t FRAMES_PER_CHUNK = 8;    // кадров в одном пакете SCAN
static constexpr size_t FRAME_SIZE = 20;

static inline float lidarAngle(uint16_t raw)
{
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360.0f) a -= 360.0f;
  return a;
}

// Ищет заголовок пакета лидара; true — найден, тело читается следующим.
static bool lidarSync()
{
  uint8_t pos = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 200) {
    if (!LIDAR_SERIAL.available()) { delay(1); continue; }
    const uint8_t b = LIDAR_SERIAL.read();
    pos = (b == LIDAR_HDR[pos]) ? pos + 1 : (b == LIDAR_HDR[0]);
    if (pos == 4) return true;
  }
  return false;
}

static bool lidarRead(uint8_t * dst, size_t n)
{
  const uint32_t t0 = millis();
  size_t got = 0;
  while (got < n) {
    if (millis() - t0 > 300) return false;
    if (!LIDAR_SERIAL.available()) { delay(1); continue; }
    got += LIDAR_SERIAL.readBytes(dst + got, n - got);
  }
  return true;
}

// Задача FreeRTOS: читает лидар и отправляет кадры кусками по FRAMES_PER_CHUNK.
// Конец оборота — когда угол начала кадра «перескакивает» через 0°.
static void lidarTask(void *)
{
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[LIDAR_BODY];
  uint8_t chunk[4 + FRAMES_PER_CHUNK * FRAME_SIZE];
  size_t frames = 0;           // кадров в текущем куске
  uint8_t chunkIdx = 0;        // номер куска в обороте
  uint16_t rev = 0;            // номер оборота
  size_t revFrames = 0;        // кадров в обороте
  uint64_t chunkStamp = 0;
  float prevStart = -1;

  auto flush = [&](bool last) {
    if (frames == 0 && !last) return;
    memcpy(chunk, &rev, 2);
    chunk[2] = chunkIdx++;
    chunk[3] = last ? proto::SCAN_FLAG_LAST : 0;
    sendPacket(proto::SCAN, chunkStamp, chunk, 4 + frames * FRAME_SIZE);
    frames = 0;
  };

  for (;;) {
    if (!lidarSync()) continue;
    const uint64_t stamp = nowUs();          // время прихода кадра
    if (!lidarRead(body, LIDAR_BODY)) continue;

    const float startDeg = lidarAngle(body[2] | (body[3] << 8));
    float endDeg = lidarAngle(body[28] | (body[29] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) continue;

    // Новый оборот: дописать хвост предыдущего с флагом «последний»
    if (prevStart >= 0 && startDeg < prevStart && revFrames >= 30) {
      flush(true);
      rev++;
      chunkIdx = 0;
      revFrames = 0;
    }
    prevStart = startDeg;

    if (frames == 0) chunkStamp = stamp;
    uint8_t * f = chunk + 4 + frames * FRAME_SIZE;
    const uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    const uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    memcpy(f, &s, 2);
    memcpy(f + 2, &e, 2);
    for (int i = 0; i < 8; ++i) {
      const uint8_t * m = body + 4 + i * 3;
      const uint16_t d = (m[2] >= INTENSITY_MIN) ? (uint16_t)(m[0] | (m[1] << 8)) : 0;
      memcpy(f + 4 + i * 2, &d, 2);
    }
    frames++;
    revFrames++;
    if (frames == FRAMES_PER_CHUNK) flush(false);
  }
}

/* ---------- Arduino ---------- */
void setup()
{
  pc.begin();
  HB_SERIAL.begin(HB_BAUD, SERIAL_8N1, HB_RX_PIN, HB_TX_PIN);
  mpuOk = initImu();
  // Лидар — отдельной задачей, чтобы ожидание его байтов не тормозило loop()
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, nullptr, 2, nullptr, 1);
}

void loop()
{
  const uint32_t now = millis();

  // Команды от ПК (может прийти несколько за итерацию)
  uint8_t rx[proto::MAX_PACKET];
  for (size_t n; (n = pc.poll(rx, sizeof(rx))) > 0;) handlePacket(rx, n);

  // Связь с ПК пропала — останавливаемся
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    tgtL = tgtR = 0;
    straightMode = false;
  }

  static uint32_t lastHb = 0;
  if (now - lastHb >= HB_CMD_PERIOD_MS) {
    lastHb = now;
    hbUpdate();
  }

  hbReadFeedback();

  static uint32_t lastOdom = 0;
  if (now - lastOdom >= ODOM_PERIOD_MS) {
    lastOdom = now;
    sendOdom();
  }

  // Раз в 5 с — состояние в лог ноды
  static uint32_t lastLog = 0;
  if (now - lastLog >= 5000) {
    lastLog = now;
    logf("alive: imu=%d link=%d", mpuOk, pc.link_up());
  }
  delay(1);
}
