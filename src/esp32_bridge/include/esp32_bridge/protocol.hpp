// Протокол ESP32 ↔ ПК, версия 2 — чистые функции и классы без ROS,
// покрыты тестами (test/test_protocol.cpp).
// Зеркальная реализация в прошивке: firmware/esp32_bridge/include/protocol.h
// (там же полное описание формата). При изменении правьте обе.
//
// Пакет: [type u8][seq u16][stamp u64][payload][crc16 Modbus], всё little-endian.
// UDP — пакет = датаграмма; USB — COBS + разделитель 0x00.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace esp32_bridge
{

namespace type
{
constexpr uint8_t kOdom = 0x01;
constexpr uint8_t kScan = 0x02;
constexpr uint8_t kPong = 0x03;
constexpr uint8_t kLog = 0x04;
constexpr uint8_t kCmd = 0x81;
constexpr uint8_t kPing = 0x82;
constexpr uint8_t kResetOdom = 0x83;
}  // namespace type

constexpr std::size_t kHeaderSize = 1 + 2 + 8;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kFrameSize = 20;         // кадр лидара: 2 угла + 8 дистанций
constexpr std::size_t kSamplesPerFrame = 8;

// CRC-16 Modbus (полином 0xA001, старт 0xFFFF) — как в прошивке.
uint16_t crc16(const uint8_t * data, std::size_t len);

struct Packet
{
  uint8_t type;
  uint16_t seq;
  uint64_t stamp;               // ESP32: мкс с загрузки
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> build_packet(uint8_t type, uint16_t seq, uint64_t stamp,
  const uint8_t * payload, std::size_t len);

// nullopt — пакет короткий или CRC не сошлась.
std::optional<Packet> parse_packet(const uint8_t * data, std::size_t len);

// ---------- COBS (кадрирование для USB) ----------
std::vector<uint8_t> cobs_encode(const std::vector<uint8_t> & in);   // без завершающего 0x00
std::optional<std::vector<uint8_t>> cobs_decode(const uint8_t * in, std::size_t len);

// Режет поток байт из USB на пакеты по разделителю 0x00.
class CobsStream
{
public:
  // Добавляет байты, возвращает декодированные пакеты (битые отбрасываются).
  std::vector<std::vector<uint8_t>> feed(const uint8_t * data, std::size_t len);

private:
  std::vector<uint8_t> buf_;
};

// ---------- Одометрия ----------
struct OdomSample
{
  float x, y, th;           // поза в кадре odom, м / рад
  float speed_left_mm;      // мм/с
  float speed_right_mm;     // мм/с
};
std::optional<OdomSample> parse_odom(const std::vector<uint8_t> & payload);

// ---------- Скан ----------
struct ScanChunk
{
  uint16_t rev;              // номер оборота
  uint8_t index;             // номер куска в обороте
  bool last;                 // последний кусок оборота
  std::vector<uint8_t> frames;   // N × kFrameSize
};
std::optional<ScanChunk> parse_scan_chunk(const std::vector<uint8_t> & payload);

// Собранный оборот (сырые кадры + время первого и последнего куска).
struct Revolution
{
  std::vector<uint8_t> frames;
  uint64_t first_stamp_us;
  uint64_t last_stamp_us;
  bool complete;             // пришли все куски подряд
};

// Собирает куски в обороты. Потерянный кусок не мешает — оборот
// публикуется с пробелом (complete = false).
class ScanAssembler
{
public:
  std::optional<Revolution> add(const ScanChunk & chunk, uint64_t stamp_us);

private:
  std::optional<uint16_t> rev_;
  uint8_t next_index_ = 0;
  Revolution cur_{};
};

// Настройки ориентации лидара (параметры ноды lidar_mirror / lidar_yaw_deg).
struct ScanOptions
{
  bool mirror = true;        // лидар вращается в обратную сторону
  double yaw_deg = 180.0;    // поворот облака, °
  std::size_t bins = 360;    // угловых ячеек на оборот
};

// Оборот, разложенный по ячейкам: ranges[i] — угол −π + i·2π/bins.
struct Scan
{
  float angle_min;
  float angle_max;
  float angle_increment;
  std::vector<float> ranges;        // м; +inf — нет измерения
  std::vector<float> intensities;   // 1 — есть измерение
};

// Раскладывает кадры по ячейкам по их настоящим углам. Если в ячейку
// попало несколько точек — берётся ближняя.
Scan bin_scan(const std::vector<uint8_t> & frames, const ScanOptions & opt);

// ---------- Синхронизация часов ----------
// Оценивает смещение часов ESP32 относительно ПК по парам PING/PONG.
// Берётся замер с наименьшим временем в пути из последних `window`:
// у него меньше всего неопределённость задержки.
class TimeSync
{
public:
  explicit TimeSync(std::size_t window = 16)
  : window_(window) {}

  // pc_send_ns — когда ПК отправил PING (он же вернулся в PONG),
  // pc_recv_ns — когда ПК получил PONG, esp_us — время ESP в PONG.
  void add(int64_t pc_send_ns, int64_t pc_recv_ns, uint64_t esp_us);

  // Время ПК (нс), соответствующее времени ESP; nullopt — замеров ещё нет.
  std::optional<int64_t> to_pc_ns(uint64_t esp_us) const;

  // Время в пути туда-обратно лучшего замера, нс (для диагностики).
  std::optional<int64_t> best_rtt_ns() const;

private:
  struct Sample
  {
    int64_t offset_ns;
    int64_t rtt_ns;
  };
  std::size_t window_;
  std::deque<Sample> samples_;
};

// ---------- Команды ----------
// Скорости колёс (мм/с) из /cmd_vel по кинематике дифф. привода.
std::array<int16_t, 2> wheel_speeds_mm(double linear_x, double angular_z, double wheel_base_m);
std::vector<uint8_t> encode_command(int16_t left, int16_t right);   // payload CMD

// Счётчик потерянных пакетов по номерам seq (с учётом переполнения u16).
class LossCounter
{
public:
  void add(uint16_t seq);
  uint64_t received() const {return received_;}
  uint64_t lost() const {return lost_;}

private:
  std::optional<uint16_t> last_;
  uint64_t received_ = 0;
  uint64_t lost_ = 0;
};

}  // namespace esp32_bridge
