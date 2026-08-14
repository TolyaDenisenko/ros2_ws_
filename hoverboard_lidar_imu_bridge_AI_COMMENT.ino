// ============================================================
// Hoverboard + LiDAR + MPU6050 IMU Bridge for ROS 2 (Jazzy)
// - Wi-Fi + HTTP + WebSocket
// - UART2: RX=18, TX=17 → hoverboard
// - LiDAR: Serial1 (RX=32, TX=33)
// - MPU6050: I2C (SDA=21, SCL=22)
//
// ИЗМЕНЕНИЯ:
//   - Одометрия больше НЕ отдаётся через HTTP /state как основной канал.
//     Вместо этого она пушится через тот же WebSocket, что и лидар,
//     с фиксированной частотой (ODOM_PERIOD_MS), с префиксом типа пакета.
//   - Оба типа пакетов (скан и одометрия) теперь имеют единый формат:
//       [1 байт тип][payload][2 байта CRC16 (Modbus) от (тип+payload)]
//     ODOM_PKT_TYPE = 0x01, SCAN_PKT_TYPE = 0x02
//   - /state оставлен для отладки через браузер/curl, но ROS-нода
//     его больше не опрашивает.
// ============================================================

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

// MPU6050 библиотека (встроенная)
#include "MPU6050.h"

/* ---------- Wi-Fi ---------- */
constexpr char WIFI_SSID[] = "Mr_sinister_";
constexpr char WIFI_PASS[] = "immortal";

/* ---------- Hoverboard UART ---------- */
#define HB_SERIAL Serial2
#define HB_BAUD 1000000

/* ---------- Параметры hoverboard'а ---------- */
const float WHEEL_BASE = 0.60f;
const float TICKS_PER_METER = 674.0f;
const float BASE_MM = WHEEL_BASE * 1000.0f;

/* ---------- Настройки лидара ---------- */
#define LIDAR_RX_PIN 32
#define LIDAR_TX_PIN 33
#define LIDAR_BAUD 115200

/* ---------- MPU6050 настройки ---------- */
MPU6050 mpu;
const int MPU6050_ADDR = 0x68;
bool mpuInitialized = false;
float yawOffset = 0.0f; // Калибровочный сдвиг
const float YAW_KP = 0.8f; // Коэффициент П-регулятора для удержания курса

static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t BODY_LEN = 32;
static const uint8_t INTENSITY_MIN = 2;
static const float MAX_SPREAD_DEG = 20.0f;
#define FRAME_LEN 20
#define MAX_FRAMES 64
static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1.0f;

/* ---------- Протокол WS: тип пакета ---------- */
static const uint8_t ODOM_PKT_TYPE = 0x01;
static const uint8_t SCAN_PKT_TYPE = 0x02;

/* ---------- Период пуша одометрии ---------- */
const unsigned long ODOM_PERIOD_MS = 20; // 50 Гц

/* ---------- Глобальные переменные состояния ---------- */
volatile float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;
volatile float speedL = 0.0f, speedR = 0.0f;
volatile float tgtL = 0.0f, tgtR = 0.0f;
volatile uint32_t lastCmdMs = 0;
const unsigned long CMD_TIMEOUT = 100;

// Режим удержания прямолинейного движения
bool straightMode = false;
float straightTargetYaw = 0.0f; // Целевой курс при движении прямо

volatile uint32_t last_time = 0;
volatile uint32_t last_imu_time = 0;

/* ---------- Статистика ---------- */
volatile uint32_t stat_rx = 0;
volatile uint32_t stat_tx = 0;
volatile uint32_t stat_odom_tx = 0;

/* ---------- Вспомогательные функции ---------- */

// decodeAngle(raw) — декодирует "сырое" 16-битное значение угла из пакета
// лидара LDS в градусы (0..360).
//   raw    — два байта из пакета лидара, как есть (little-endian uint16)
//   return — угол в градусах, float, диапазон [0, 360)
// Формат кодирования специфичен для этой модели LDS: смещение 0xA000
// и масштаб 1/64 градуса на единицу — взято из протокола датчика.
inline float decodeAngle(uint16_t raw) {
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360) a -= 360.0f;
  return a;
}

// readBytes(serial, dst, n, timeout) — блокирующее чтение ровно n байт
// из указанного HardwareSerial с ограничением по времени.
//   serial  — ссылка на UART-порт (например, Serial1 для лидара)
//   dst     — куда записать прочитанные байты (буфер должен вмещать n)
//   n       — сколько байт нужно прочитать
//   timeout — сколько миллисекунд ждать данные, прежде чем сдаться (по умолчанию 300 мс)
//   return  — true, если удалось прочитать все n байт; false — по таймауту
// Внутри крутится delay(1) в ожидании байт — так что вызывающий код
// (lidarTask) на время вызова блокируется, но т.к. это отдельная FreeRTOS-
// задача на своём ядре, на остальную систему это не влияет.
bool readBytes(HardwareSerial &serial, uint8_t *dst, size_t n, uint32_t timeout = 300) {
  uint32_t t0 = millis();
  for (size_t i = 0; i < n; ++i) {
    while (!serial.available()) {
      if (millis() - t0 > timeout) return false;
      delay(1);
    }
  }
  serial.readBytes(dst, n);
  return true;
}

// waitLidarHeader(serial) — ищет в потоке байт 4-байтовую сигнатуру
// заголовка пакета лидара (HDR = {0x55, 0xAA, 0x03, 0x08}) и
// синхронизируется на неё, отбрасывая всё до совпадения.
//   serial — ссылка на UART-порт лидара
//   return — true, если заголовок найден за отведённые 200 мс; false — если нет
// Нужно, потому что UART не даёт границ пакетов "из коробки" — без
// такой посимвольной синхронизации после сбоя/рассинхрона можно
// надолго "залипнуть" на кривом смещении в потоке.
bool waitLidarHeader(HardwareSerial &serial) {
  uint8_t pos = 0;
  uint32_t t0 = millis();
  while (true) {
    if (serial.available()) {
      if (uint8_t(serial.read()) == HDR[pos]) {
        if (++pos == 4) return true;
      } else {
        pos = 0;
      }
    }
    if (millis() - t0 > 200) return false;
  }
}

// crc16(crc, v) — один шаг вычисления CRC-16 (алгоритм Modbus,
// полином 0xA001, обратный порядок бит). Вызывается по одному байту
// за раз, накопленное значение crc передаётся между вызовами.
//   crc    — текущее накопленное значение CRC (начинать с 0xFFFF)
//   v      — очередной байт данных
//   return — обновлённое значение CRC после обработки этого байта
// Используется и на ESP (при формировании исходящих WS-пакетов), и
// на стороне Python-ноды (при проверке) — важно, чтобы обе стороны
// использовали идентичный алгоритм, иначе пакеты будут браковаться.
inline uint16_t crc16(uint16_t crc, uint8_t v) {
  crc ^= v;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ============================================================
// initMPU6050() — инициализация и калибровка IMU-датчика MPU6050.
//   Вызывается один раз из setup().
//   return — true, если датчик найден на шине I2C и отвечает
//            (mpu.testConnection() == true); false, если связи нет
//            (в этом случае mpuInitialized останется false, и все
//            расчёты курса через getIMUYaw() будут возвращать 0).
// Что делает:
//   - Поднимает шину I2C на пинах SDA=21, SCL=22
//   - Запускает калибровку гироскопа и акселерометра (mpu.CalibrateGyro/
//     CalibrateAccel — библиотечные функции, вращают несколько итераций
//     сбора смещения нуля; робот в этот момент должен стоять неподвижно)
//   - Выставляет диапазоны чувствительности гироскопа (±500°/с) и
//     акселерометра (±4g) — см. константы MPU6050_GYRO_FS_500 /
//     MPU6050_ACCEL_FS_4 из библиотеки MPU6050.h
//   - Делает один холостой замер mpu.getMotion6() для "прогрева"
// ============================================================
bool initMPU6050() {
  Wire.begin(21, 22); // SDA=21, SCL=22 для ESP32-S3
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    return false;
  }

  Serial.println("MPU6050 initialized!");

  // Калибровка гироскопа
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);

  // Установка чувствительности
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  delay(100);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  yawOffset = 0.0f;

  return true;
}

// ============================================================
// getIMUYaw() — читает текущий курс (yaw) робота по гироскопу MPU6050.
//   return — курс в радианах, нормализован в диапазон [-π, π].
//            Если IMU не был успешно инициализирован (mpuInitialized
//            == false), всегда возвращает 0.0.
// Как считается:
//   Это ПРОСТОЕ ИНТЕГРИРОВАНИЕ угловой скорости по оси Z (гироскоп),
//   без акселерометра/магнитометра и без фильтра (Маджвика/Калмана).
//   На каждый вызов берётся dt с прошлого вызова, читается gz,
//   переводится в рад/с через коэффициент чувствительности датчика
//   (для диапазона ±500°/с: 65.5 LSB/(°/с) = 3754.5 LSB/(рад/с)),
//   и накапливается yaw += gz_rad * dt.
//   ВАЖНО: такое интегрирование подвержено дрейфу — со временем
//   накопленная ошибка растёт, особенно при неточной калибровке нуля
//   гироскопа. Для длительной работы / точной локализации стоит
//   рассмотреть комплементарный фильтр или полноценный AHRS
//   (например, Madgwick), учитывающий акселерометр.
//   Функция вызывается достаточно часто — из update_hoverboard_command()
//   (в режиме удержания прямой) и из onWsEvent()/HTTP setSpeed при
//   входе в straightMode — поэтому dt между вызовами непостоянен.
// ============================================================
float getIMUYaw() {
  static float yaw = 0.0f;

  if (!mpuInitialized) return 0.0f;

  unsigned long now = millis();
  float dt = (now - last_imu_time) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  last_imu_time = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // MPU6050_GYRO_FS_500 = 500 dps → 65.5 LSB/(deg/s) = 3754.5 LSB/(rad/s)
  float gyro_z_rad = gz / 3754.5f;

  yaw += gyro_z_rad * dt;

  while (yaw > M_PI) yaw -= 2 * M_PI;
  while (yaw < -M_PI) yaw += 2 * M_PI;

  return yaw;
}

// ============================================================
// send_command(steer, speed) — низкоуровневая отправка команды
// управления в контроллер hoverboard'а через UART (HB_SERIAL).
//   steer  — команда поворота (разница скоростей колёс), диапазон
//            протокола примерно [-1000, 1000]
//   speed  — команда линейной скорости, диапазон примерно [-1000, 1000]
// Формат кадра (8 байт), специфичный для прошивки hoverboard-hack:
//   [0]   0xCD  — байт-маркер начала кадра
//   [1]   0xAB  — второй байт-маркер
//   [2-3] steer — int16 little-endian
//   [4-5] speed — int16 little-endian
//   [6-7] checksum — младший/старший байт XOR (0xABCD ^ steer ^ speed)
// Проверку допустимого диапазона (constrain) эта функция НЕ делает —
// ожидается, что вызывающий код (update_hoverboard_command) уже
// подрезал значения перед вызовом.
// ============================================================
void send_command(int16_t steer, int16_t speed) {
    uint8_t buf[8];
    buf[0] = 0xCD;
    buf[1] = 0xAB;
    memcpy(&buf[2], &steer, 2);
    memcpy(&buf[4], &speed, 2);
    uint16_t cs = 0xABCD ^ steer ^ speed;
    buf[6] = cs & 0xFF;
    buf[7] = (cs >> 8) & 0xFF;
    HB_SERIAL.write(buf, 8);
}

// ============================================================
// update_hoverboard_command() — главный цикл управления мотором:
// превращает целевые скорости колёс (tgtL/tgtR, приходят из
// onWsEvent()/HTTP setSpeed в мм/с) в команды steer/speed для
// send_command(), с опциональной IMU-коррекцией курса.
//   Параметров нет, работает с глобальными переменными tgtL, tgtR,
//   straightMode, straightTargetYaw.
//   Вызывается из loop() каждые 50 мс (см. last_send).
// Логика:
//   1. tgtL/tgtR (мм/с) переводятся в м/с (v_left, v_right)
//   2. linear_vel = среднее двух колёс, angular_vel = их разница / базу
//   3. Если straightMode включён (см. onWsEvent — активируется, когда
//      левая и правая целевые скорости почти равны и не нулевые) и
//      линейная скорость заметна (> 0.05 м/с):
//        - берём текущий курс с IMU (getIMUYaw)
//        - считаем ошибку между straightTargetYaw (курс, зафиксированный
//          в момент начала прямолинейного движения) и текущим курсом
//        - применяем П-регулятор (YAW_KP) с ограничением коррекции
//          [-0.5, 0.5] рад/с и добавляем её к angular_vel
//      Это компенсирует небольшой увод робота в сторону при движении
//      "прямо", вызванный несимметрией колёс/моторов.
//   4. angular_vel и linear_vel масштабируются в единицы протокола
//      hoverboard'а (эмпирические коэффициенты 400.0 и 600.0) и
//      обрезаются в диапазон [-1000, 1000]
//   5. Результат отправляется через send_command()
// ============================================================
void update_hoverboard_command() {
    float v_left = tgtL / 1000.0f;
    float v_right = tgtR / 1000.0f;

    float linear_vel = (v_left + v_right) * 0.5f;
    float angular_vel = (v_left - v_right) / WHEEL_BASE;

    if (straightMode && fabs(linear_vel) > 0.05f) {
        float currentYaw = getIMUYaw();
        float yawError = straightTargetYaw - currentYaw;

        while (yawError > M_PI) yawError -= 2 * M_PI;
        while (yawError < -M_PI) yawError += 2 * M_PI;

        float correction = yawError * YAW_KP;
        correction = constrain(correction, -0.5f, 0.5f);

        angular_vel += correction;
    }

    int16_t steer = (int16_t)(angular_vel * 400.0f);
    int16_t speed = (int16_t)(linear_vel * 600.0f);
    steer = constrain(steer, -1000, 1000);
    speed = constrain(speed, -1000, 1000);

    send_command(steer, speed);
}

// ============================================================
// WebSocket
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient = nullptr;

// ============================================================
// onWsEvent(...) — колбэк AsyncWebSocket на ВСЕ события WS: подключение
// клиента, отключение, входящие данные. Регистрируется один раз в
// setup() через ws.onEvent(onWsEvent).
//   server — указатель на объект AsyncWebSocket (сам "/ws"-эндпоинт)
//   client — указатель на конкретного клиента, к которому относится событие
//   type   — тип события: WS_EVT_CONNECT / WS_EVT_DISCONNECT / WS_EVT_DATA / ...
//   arg    — доп. данные события; для WS_EVT_DATA это AwsFrameInfo*
//            (метаданные фрейма — opcode, финальность, длина и т.п.)
//   data   — указатель на байты полезной нагрузки (для WS_EVT_DATA)
//   len    — длина payload в байтах (для WS_EVT_DATA)
// Ветки:
//   WS_EVT_CONNECT — сохраняем указатель на клиента в глобальный
//     wsClient (см. дизайн ниже) и отключаем алгоритм Нейгла
//     (setNoDelay(true)) для минимальной задержки мелких пакетов.
//     Поддерживается ТОЛЬКО ОДИН активный клиент одновременно: если
//     кто-то уже подключён — его соединение принудительно закрывается
//     перед тем, как принять новое.
//   WS_EVT_DISCONNECT — если отключившийся клиент был тем самым
//     wsClient — обнуляем указатель, чтобы sendOdomPacket()/lidarTask
//     не пытались слать данные в "мёртвый" сокет.
//   WS_EVT_DATA — обрабатываем входящие команды скорости от ROS-ноды.
//     Ожидается БИНАРНЫЙ фрейм РОВНО 4 байта:
//       data[0..1] = left  (int16, little-endian, мм/с)
//       data[2..3] = right (int16, little-endian, мм/с)
//     Это тот самый канал, которым Python-нода теперь шлёт /cmd_vel
//     вместо HTTP GET /setSpeed — экономит на накладных расходах
//     HTTP и не плодит новые TCP-соединения на каждую команду.
//     После разбора: обновляются tgtL/tgtR, lastCmdMs (для watchdog-
//     таймаута CMD_TIMEOUT в loop()), и решается, включать ли
//     straightMode (когда |L|≈|R| и обе заметно ненулевые — считаем,
//     что просят ехать прямо, и фиксируем текущий курс как целевой
//     для IMU-коррекции в update_hoverboard_command()).
// ============================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (wsClient && wsClient->status() == WS_CONNECTED) {
      wsClient->close();
    }
    wsClient = client;
    wsClient->client()->setNoDelay(true);
    Serial.printf("[WS] Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == 4) {
      int16_t left = data[0] | (data[1] << 8);
      int16_t right = data[2] | (data[3] << 8);
      tgtL = (float)left;
      tgtR = (float)right;
      lastCmdMs = millis();

      if (fabs(fabs(tgtL) - fabs(tgtR)) < 10.0f && fabs(tgtL) > 10.0f) {
        straightMode = true;
        straightTargetYaw = getIMUYaw();
      } else {
        straightMode = false;
      }
      Serial.printf("[WS] Cmd: L=%d, R=%d\n", left, right);
    }
  }
}

// ============================================================
// sendOdomPacket() — формирует и отправляет через WebSocket один
// пакет с текущей одометрией. Вызывается из loop() с фиксированным
// периодом ODOM_PERIOD_MS (по умолчанию 20 мс → 50 Гц).
//   Параметров нет, читает глобальные odomX/odomY/odomTh/speedL/speedR.
// Формат пакета на проводе (23 байта): [1][20][2]
//   [0]     тип пакета = ODOM_PKT_TYPE (0x01)
//   [1-20]  payload — 5 значений float32 little-endian подряд:
//           x (м), y (м), th (рад), speedL (мм/с), speedR (мм/с)
//   [21-22] CRC16 (Modbus) от байтов [0..20] включительно (тип+payload),
//           little-endian
// Ничего не делает (тихо выходит), если:
//   - клиент не подключён (wsClient == nullptr)
//   - клиент подключён, но не готов принимать данные (canSend() == false,
//     например, буфер TCP-отправки ещё не освободился с прошлого кадра)
// Такой pull-through подход (просто пропустить кадр, если получатель
// не готов) — сознательный выбор: одометрия и так шлётся часто
// (50 Гц), пропуск одного кадра не критичен, а вот блокировка loop()
// в ожидании готовности сокета была бы гораздо хуже.
// ============================================================
void sendOdomPacket() {
  if (!wsClient || !wsClient->canSend()) return;

  const size_t PAYLOAD_SZ = 5 * sizeof(float);
  uint8_t pkt[1 + PAYLOAD_SZ + 2];

  pkt[0] = ODOM_PKT_TYPE;

  float vals[5] = { odomX, odomY, odomTh, speedL, speedR };
  memcpy(&pkt[1], vals, PAYLOAD_SZ);

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < 1 + PAYLOAD_SZ; ++i) crc = crc16(crc, pkt[i]);
  pkt[1 + PAYLOAD_SZ] = crc & 0xFF;
  pkt[1 + PAYLOAD_SZ + 1] = crc >> 8;

  wsClient->binary(pkt, sizeof(pkt));
  stat_odom_tx++;
}

// ============================================================
// setupRoutes() — регистрирует все HTTP-маршруты на AsyncWebServer.
// Вызывается один раз из setup(), до server.begin().
//   Параметров нет. Все обработчики — лямбды, захватывающие глобальное
//   состояние (speedL/speedR/tgtL/tgtR/odomX/Y/Th) по значению момента
//   вызова (т.к. это глобальные переменные, каждый раз читается
//   актуальное значение).
// ВАЖНО: начиная с этой версии ROS-нода общается с платой ИСКЛЮЧИТЕЛЬНО
// через WebSocket ("/ws") — и для получения данных (скан+одометрия), и
// для отправки команд скорости. Маршруты ниже — это debug/legacy-
// интерфейс для ручной проверки через браузер или curl, в горячем
// пути управления они больше не участвуют.
//
// Маршруты:
//   GET /state
//     Отдаёт JSON-снимок текущего состояния (скорости, цели, одометрию).
//     Поля duty/enc всегда нулевые — исторический остаток формата,
//     реальных данных туда сейчас не пишется.
//     Пример: curl http://<esp-ip>/state
//
//   GET /setSpeed?l=<int>&r=<int>
//     Ручная установка целевых скоростей колёс в мм/с (та же логика,
//     что и в onWsEvent для бинарных команд — включая straightMode).
//     Удобно для быстрой проверки моторов без поднятия ROS-стека.
//     Пример: curl "http://<esp-ip>/setSpeed?l=300&r=300"
//
//   GET /resetOdom
//     Обнуляет накопленную одометрию (odomX/Y/Th = 0). Полезно перед
//     началом нового теста, чтобы не тащить дрейф с прошлого запуска.
//
//   GET /
//     Простая текстовая справка по доступным маршрутам.
// ============================================================
void setupRoutes() {
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    char json[512];
    snprintf(json, sizeof(json),
             "{\"duty\":{\"L_A\":0,\"L_B\":0,\"R_A\":0,\"R_B\":0},"
             "\"enc\":{\"left\":0,\"right\":0},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f}}",
             speedL, speedR, tgtL, tgtR, odomX, odomY, odomTh);
    request->send(200, "application/json", json);
  });

  server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("l")) tgtL = request->getParam("l")->value().toFloat();
    if (request->hasParam("r")) tgtR = request->getParam("r")->value().toFloat();
    lastCmdMs = millis();

    if (fabs(fabs(tgtL) - fabs(tgtR)) < 10.0f && fabs(tgtL) > 10.0f) {
      straightMode = true;
      straightTargetYaw = getIMUYaw();
    } else {
      straightMode = false;
    }
    request->send(200, "text/plain", "ok");
  });

  server.on("/resetOdom", HTTP_GET, [](AsyncWebServerRequest *request) {
    odomX = odomY = odomTh = 0;
    request->send(200, "text/plain", "odom reset");
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hoverboard+LiDAR+IMU: /state (debug), /setSpeed, /resetOdom, /ws (scan+odom push)");
  });
}

// ============================================================
// lidarTask(param) — отдельная FreeRTOS-задача, читающая поток данных
// с лидара (Serial1) и собирающая их в пакеты для отправки по WS.
// Запускается один раз из setup() через xTaskCreatePinnedToCore,
// закреплена за ядром 1 (см. комментарий в setup()), крутится
// бесконечно (while(true)) до перезагрузки платы.
//   param — не используется (обязательный параметр сигнатуры FreeRTOS-задачи)
// Алгоритм на каждой итерации:
//   1. delay(1) — отдать процессор другим задачам того же приоритета
//   2. waitLidarHeader() — синхронизируемся на начало следующего
//      "тела" пакета лидара (4-байтовая сигнатура HDR)
//   3. readBytes() — читаем оставшиеся BODY_LEN (32) байт тела:
//      начальный угол, 8 троек (дистанция+качество), конечный угол
//   4. Декодируем startDeg/endDeg через decodeAngle(); если угловой
//      охват этого "тела" подозрительно большой (> MAX_SPREAD_DEG,
//      признак сбоя/рассинхрона) — отбрасываем и читаем дальше
//   5. Упаковываем 8 замеров (дистанция обнуляется, если качество
//      сигнала quality[i] < INTENSITY_MIN — считаем измерение
//      недостоверным) в компактный 20-байтовый "кадр" и дописываем
//      его в scanBuf (глобальный кольцевой-ish буфер, указатель wr
//      двигается вперёд с каждым кадром)
//   6. Если буфер переполнился (frameCount >= MAX_FRAMES) — считаем,
//      что что-то пошло не так (за это время должен был случиться
//      полный оборот), сбрасываем накопление и начинаем заново
//   7. Определяем конец полного оборота лидара по признаку "угол
//      пошёл на новый круг" (startDeg меньше предыдущего prevStartAngle)
//      при разумном минимальном числе накопленных кадров (>= 30, чтобы
//      не улавливать случайный шум на старте). Если оборот завершён:
//        - копируем накопленные кадры (scanBuf) в исходящий буфер
//          txBuf после однобайтового префикса типа (SCAN_PKT_TYPE)
//        - считаем CRC16 по (тип + все кадры)
//        - если клиент подключён и готов принимать — отправляем
//          весь пакет одним WS-бинарным фреймом
//        - сбрасываем накопление для следующего оборота
// Буфер txBuf объявлен static внутри функции — выделяется один раз
// на весь срок жизни задачи, а не на стеке при каждом вызове, что
// экономит стек задачи (4096 байт, см. xTaskCreatePinnedToCore) —
// иначе scanBuf-размера (64*20 байт) локальный массив рисковал бы
// его переполнить.
// ============================================================
void lidarTask(void *param) {
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[BODY_LEN];

  // Буфер под исходящий пакет: 1 байт тип + весь scanBuf + 2 байта CRC
  static uint8_t txBuf[1 + MAX_FRAMES * FRAME_LEN + 2];
  txBuf[0] = SCAN_PKT_TYPE;

  while (true) {
    delay(1);
    if (!waitLidarHeader(Serial1)) continue;
    if (!readBytes(Serial1, body, BODY_LEN)) continue;

    float startDeg = decodeAngle(body[2] | (body[3] << 8));
    uint8_t offset = 4;
    uint16_t dist[8];
    uint8_t quality[8];
    for (int i = 0; i < 8; ++i) {
      dist[i] = body[offset] | (body[offset + 1] << 8);
      quality[i] = body[offset + 2];
      offset += 3;
    }
    float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) continue;

    uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    *wr++ = s & 0xFF;
    *wr++ = s >> 8;
    *wr++ = e & 0xFF;
    *wr++ = e >> 8;
    for (int i = 0; i < 8; ++i) {
      uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
      *wr++ = d & 0xFF;
      *wr++ = d >> 8;
    }
    frameCount++;
    stat_rx++;

    if (frameCount >= MAX_FRAMES) {
      frameCount = 0;
      wr = scanBuf;
      prevStartAngle = -1.0f;
      continue;
    }

    if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
      size_t scanSize = frameCount * FRAME_LEN;

      memcpy(&txBuf[1], scanBuf, scanSize);

      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < 1 + scanSize; ++i) crc = crc16(crc, txBuf[i]);
      txBuf[1 + scanSize] = crc & 0xFF;
      txBuf[1 + scanSize + 1] = crc >> 8;

      if (wsClient && wsClient->canSend()) {
        wsClient->binary(txBuf, 1 + scanSize + 2);
        stat_tx++;
      }
      wr = scanBuf;
      frameCount = 0;
    }
    prevStartAngle = startDeg;
  }
}

// ============================================================
// setup() — стандартная точка входа Arduino, выполняется один раз
// при старте/перезагрузке платы.
// Порядок инициализации важен:
//   1. Serial (для отладочного вывода) на 1 000 000 бод
//   2. MPU6050 (initMPU6050) — если не найден, работа продолжается,
//      но mpuInitialized останется false и IMU-коррекция курса будет
//      отключена (getIMUYaw всегда вернёт 0)
//   3. UART к hoverboard'у (HB_SERIAL) на 1 000 000 бод, пины RX=18/TX=17
//   4. Wi-Fi: подключение в блокирующем цикле (delay(300) до
//      WL_CONNECTED) — то есть плата не продолжит инициализацию,
//      пока не подключится к сети. WiFi.setSleep(false) отключает
//      энергосбережение модема — критично для низкой и стабильной
//      задержки WS/HTTP-трафика (со sleep включённым модем может
//      надолго "засыпать" между пакетами)
//   5. Регистрация обработчика WS-событий, HTTP-маршрутов, старт
//      сервера
//   6. Запуск lidarTask как отдельной FreeRTOS-задачи на ядре 1,
//      с приоритетом 2 и стеком 4096 байт — чтобы не конкурировать
//      за CPU с сетевым стеком AsyncTCP, который по умолчанию сидит
//      на ядре 0
//   7. Инициализация меток времени (last_time/last_imu_time) для
//      последующих расчётов dt
// ============================================================
void setup() {
  Serial.begin(1000000);
  Serial.println("=== Hoverboard + LiDAR + MPU6050 Bridge ===");

  if (initMPU6050()) {
    mpuInitialized = true;
    Serial.println("MPU6050 ready!");
  } else {
    Serial.println("MPU6050 failed!");
  }

  HB_SERIAL.begin(HB_BAUD, SERIAL_8N1, 18, 17);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  setupRoutes();
  server.begin();

  // Лидар — на ядре 1, чтобы не конкурировать с AsyncTCP (обычно ядро 0)
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, nullptr, 2, nullptr, 1);

  last_time = millis();
  last_imu_time = millis();
}

// ============================================================
// loop() — основной цикл Arduino, выполняется непрерывно после setup().
// Здесь НЕ обрабатывается лидар (это отдельная задача lidarTask на
// ядре 1) — loop() занимается управлением моторами, чтением
// телеметрии hoverboard'а, интегрированием одометрии и её пушем по WS.
// На каждой итерации:
//   1. Watchdog по командам: если с последней полученной команды
//      скорости (lastCmdMs, обновляется в onWsEvent/HTTP setSpeed)
//      прошло больше CMD_TIMEOUT (100 мс) — обнуляем целевые скорости
//      и выключаем straightMode. Это защита от "убегания" робота,
//      если связь с ROS-нодой оборвалась.
//   2. Отправка команды в hoverboard: не чаще одного раза в 50 мс
//      (20 Гц) вызывается update_hoverboard_command() — эта частота
//      определяет и максимальную частоту ответной телеметрии, если
//      прошивка hoverboard'а отвечает синхронно на каждую команду.
//   3. Чтение телеметрии от hoverboard'а: если накопилось >= 18 байт
//      во входном буфере HB_SERIAL — читаем кадр целиком и, если
//      маркер начала (0xCD 0xAB) на месте, разбираем измеренные
//      скорости колёс (speedL_meas/speedR_meas, в "тиках", переводим
//      в м/с через TICKS_PER_METER). Знак speedL_meas инвертируется —
//      компенсация физической ориентации левого мотора/энкодера.
//      На основе измеренных (не целевых!) скоростей интегрируется
//      одометрия по простой diff-drive кинематике:
//        odomX += v * cos(th) * dt;  odomY += v * sin(th) * dt;
//        odomTh += w * dt;
//      с нормализацией odomTh в [-π, π].
//   4. Пуш одометрии по WS: не чаще раза в ODOM_PERIOD_MS (20 мс,
//      50 Гц) вызывается sendOdomPacket() — независимо от того,
//      пришла ли в этом цикле новая телеметрия от hoverboard'а
//      (шлётся последнее известное состояние).
//   5. ws.cleanupClients() — служебная уборка AsyncWebSocket
//      (закрытие мёртвых соединений и т.п.), рекомендуется вызывать
//      регулярно согласно документации библиотеки.
//   6. delay(1) — короткая пауза, отдать процессор другим задачам/
//      прерываниям на этом ядре (ядро 0, где также крутится Wi-Fi/
//      AsyncTCP).
// ============================================================
void loop() {
  unsigned long now = millis();

  if (now - lastCmdMs > CMD_TIMEOUT) {
    tgtL = tgtR = 0;
    straightMode = false;
  }

  static unsigned long last_send = 0;
  if (now - last_send >= 50) {
    update_hoverboard_command();
    last_send = now;
  }

  // Чтение телеметрии от hoverboard'а
  if (HB_SERIAL.available() >= 18) {
    uint8_t buf[18];
    HB_SERIAL.readBytes(buf, 18);
    if (buf[0] == 0xCD && buf[1] == 0xAB) {
      int16_t speedR_meas, speedL_meas;
      memcpy(&speedR_meas, &buf[6], 2);
      memcpy(&speedL_meas, &buf[8], 2);

      speedL_meas = -speedL_meas; // инверсия колеса

      float dt = (now - last_time) / 1000.0f;
      last_time = now;

      float v_left_m = (float)speedL_meas / TICKS_PER_METER;
      float v_right_m = (float)speedR_meas / TICKS_PER_METER;
      speedL = v_left_m * 1000.0f;
      speedR = v_right_m * 1000.0f;

      float linear_vel = (v_left_m + v_right_m) * 0.5f;
      float angular_vel = (v_right_m - v_left_m) / WHEEL_BASE;

      odomX += linear_vel * cosf(odomTh) * dt;
      odomY += linear_vel * sinf(odomTh) * dt;
      odomTh += angular_vel * dt;

      while (odomTh > M_PI) odomTh -= 2 * M_PI;
      while (odomTh < -M_PI) odomTh += 2 * M_PI;
    }
  }

  // Пуш одометрии через WS с фиксированной частотой
  static unsigned long last_odom_send = 0;
  if (now - last_odom_send >= ODOM_PERIOD_MS) {
    sendOdomPacket();
    last_odom_send = now;
  }

  ws.cleanupClients();
  delay(1);
}
