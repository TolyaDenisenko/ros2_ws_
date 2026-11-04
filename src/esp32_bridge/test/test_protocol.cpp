// Юнит-тесты протокола v2 (без сети и без ROS).
// Часть тестов собирает пакеты кодом ПРОШИВКИ (firmware/esp32_bridge/include/protocol.h)
// и разбирает кодом ноды — так ловится рассинхрон форматов между сторонами.
// Запуск: colcon test --packages-select esp32_bridge && colcon test-result --verbose

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "esp32_bridge/protocol.hpp"
#include "protocol.h"   // прошивка: namespace proto

using namespace esp32_bridge;

namespace
{

void put_u16(std::vector<uint8_t> & v, uint16_t x)
{
  v.push_back(x & 0xFF);
  v.push_back(x >> 8);
}

// Кадр лидара: углы в градусах, все 8 дистанций = mm
void put_frame(std::vector<uint8_t> & v, double start_deg, double end_deg, uint16_t mm)
{
  put_u16(v, static_cast<uint16_t>(start_deg * 100));
  put_u16(v, static_cast<uint16_t>(end_deg * 100));
  for (int j = 0; j < 8; ++j) {
    put_u16(v, mm);
  }
}

// Пакет, собранный кодом прошивки
std::vector<uint8_t> fw_packet(uint8_t t, uint16_t seq, uint64_t stamp, const std::vector<uint8_t> & p)
{
  std::vector<uint8_t> out(proto::MAX_PACKET);
  out.resize(proto::build(out.data(), t, seq, stamp, p.data(), p.size()));
  return out;
}

ScanChunk chunk(uint16_t rev, uint8_t idx, bool last, int frames = 1)
{
  ScanChunk c{rev, idx, last, {}};
  for (int i = 0; i < frames; ++i) {
    put_frame(c.frames, 0, 7, 1000);
  }
  return c;
}

}  // namespace

// ---------- CRC и пакеты ----------

TEST(Crc16, ModbusReferenceVector)
{
  const char * s = "123456789";   // эталон CRC-16/MODBUS — 0x4B37
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t *>(s), 9), 0x4B37);
  EXPECT_EQ(proto::crc16(reinterpret_cast<const uint8_t *>(s), 9), 0x4B37);
}

TEST(Packet, FirmwareBuiltParsedByNode)
{
  const auto p = fw_packet(type::kOdom, 0x1234, 0x0102030405060708ULL, {9, 8, 7});
  const auto pkt = parse_packet(p.data(), p.size());
  ASSERT_TRUE(pkt);
  EXPECT_EQ(pkt->type, type::kOdom);
  EXPECT_EQ(pkt->seq, 0x1234);
  EXPECT_EQ(pkt->stamp, 0x0102030405060708ULL);
  EXPECT_EQ(pkt->payload, (std::vector<uint8_t>{9, 8, 7}));
}

TEST(Packet, NodeBuiltParsedByFirmware)
{
  const auto cmd = encode_command(-300, 450);
  const auto p = build_packet(type::kCmd, 7, 123456789, cmd.data(), cmd.size());
  uint8_t t;
  uint64_t stamp;
  const uint8_t * payload;
  size_t len;
  ASSERT_TRUE(proto::parse(p.data(), p.size(), t, stamp, payload, len));
  EXPECT_EQ(t, proto::CMD);
  EXPECT_EQ(stamp, 123456789u);
  ASSERT_EQ(len, 4u);
  int16_t l, r;
  std::memcpy(&l, payload, 2);
  std::memcpy(&r, payload + 2, 2);
  EXPECT_EQ(l, -300);
  EXPECT_EQ(r, 450);
}

TEST(Packet, CorruptedRejected)
{
  auto p = fw_packet(type::kScan, 1, 2, {1, 2, 3});
  p[5] ^= 0x10;
  EXPECT_FALSE(parse_packet(p.data(), p.size()));
}

TEST(Packet, TooShortRejected)
{
  const uint8_t p[5] = {1, 2, 3, 4, 5};
  EXPECT_FALSE(parse_packet(p, sizeof(p)));
}

// ---------- COBS ----------

TEST(Cobs, RoundTripWithZerosAndLongRuns)
{
  std::vector<uint8_t> data = {0, 1, 0, 0, 2};
  for (int i = 0; i < 600; ++i) {
    data.push_back(static_cast<uint8_t>(i % 255 + 1));   // >254 ненулевых подряд
  }
  data.push_back(0);
  const auto enc = cobs_encode(data);
  EXPECT_EQ(std::count(enc.begin(), enc.end(), 0), 0);
  const auto dec = cobs_decode(enc.data(), enc.size());
  ASSERT_TRUE(dec);
  EXPECT_EQ(*dec, data);
}

TEST(Cobs, CompatibleWithFirmware)
{
  const std::vector<uint8_t> data = {0x11, 0x00, 0x22, 0x00, 0x00, 0x33};
  std::vector<uint8_t> fw(16);
  fw.resize(proto::cobs_encode(data.data(), data.size(), fw.data()));
  EXPECT_EQ(fw, cobs_encode(data));
  std::vector<uint8_t> back(16);
  back.resize(proto::cobs_decode(fw.data(), fw.size(), back.data()));
  EXPECT_EQ(back, data);
}

TEST(CobsStream, SplitsAndSurvivesGarbage)
{
  const auto a = fw_packet(type::kLog, 1, 0, {'h', 'i'});
  const auto b = fw_packet(type::kOdom, 2, 0, {0, 0, 0});
  std::vector<uint8_t> stream = {0x55, 0x66};   // мусор до первого разделителя
  stream.push_back(0);
  for (const auto * p : {&a, &b}) {
    auto e = cobs_encode(*p);
    stream.insert(stream.end(), e.begin(), e.end());
    stream.push_back(0);
  }
  CobsStream cs;
  // Подаём по 3 байта — как будто читаем порт маленькими порциями
  std::vector<std::vector<uint8_t>> got;
  for (std::size_t i = 0; i < stream.size(); i += 3) {
    auto part = cs.feed(stream.data() + i, std::min<std::size_t>(3, stream.size() - i));
    got.insert(got.end(), part.begin(), part.end());
  }
  std::vector<std::vector<uint8_t>> valid;
  for (auto & g : got) {
    if (parse_packet(g.data(), g.size())) {
      valid.push_back(g);
    }
  }
  ASSERT_EQ(valid.size(), 2u);
  EXPECT_EQ(valid[0], a);
  EXPECT_EQ(valid[1], b);
}

// ---------- Одометрия ----------

TEST(Odom, ParsesFiveFloats)
{
  const float v[5] = {1.5f, -2.0f, 0.25f, 100.0f, 120.0f};
  std::vector<uint8_t> p(sizeof(v));
  std::memcpy(p.data(), v, sizeof(v));   // как memcpy(float[5]) в прошивке
  const auto o = parse_odom(p);
  ASSERT_TRUE(o);
  EXPECT_FLOAT_EQ(o->x, 1.5f);
  EXPECT_FLOAT_EQ(o->th, 0.25f);
  EXPECT_FLOAT_EQ(o->speed_right_mm, 120.0f);
  EXPECT_FALSE(parse_odom(std::vector<uint8_t>(19)));
}

// ---------- Скан ----------

TEST(ScanChunk, Parses)
{
  std::vector<uint8_t> p = {0x34, 0x12, 5, 1};
  put_frame(p, 10, 17, 500);
  const auto c = parse_scan_chunk(p);
  ASSERT_TRUE(c);
  EXPECT_EQ(c->rev, 0x1234);
  EXPECT_EQ(c->index, 5);
  EXPECT_TRUE(c->last);
  EXPECT_EQ(c->frames.size(), kFrameSize);
  p.push_back(0);   // длина не кратна кадру
  EXPECT_FALSE(parse_scan_chunk(p));
}

TEST(ScanAssembler, CompleteRevolution)
{
  ScanAssembler a;
  EXPECT_FALSE(a.add(chunk(3, 0, false, 2), 1000));
  EXPECT_FALSE(a.add(chunk(3, 1, false, 2), 1100));
  const auto r = a.add(chunk(3, 2, true, 1), 1200);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->complete);
  EXPECT_EQ(r->frames.size(), 5 * kFrameSize);
  EXPECT_EQ(r->first_stamp_us, 1000u);
  EXPECT_EQ(r->last_stamp_us, 1200u);
}

TEST(ScanAssembler, LostMiddleChunkMarksIncomplete)
{
  ScanAssembler a;
  a.add(chunk(1, 0, false), 0);
  const auto r = a.add(chunk(1, 2, true), 10);   // кусок 1 потерян
  ASSERT_TRUE(r);
  EXPECT_FALSE(r->complete);
}

TEST(ScanAssembler, LostLastChunkDropsRevolution)
{
  ScanAssembler a;
  a.add(chunk(1, 0, false), 0);
  a.add(chunk(1, 1, false), 10);   // последний кусок оборота 1 потерян
  EXPECT_FALSE(a.add(chunk(2, 0, false), 20));
  const auto r = a.add(chunk(2, 1, true), 30);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->complete);
  EXPECT_EQ(r->first_stamp_us, 20u);   // старт оборота 2, без хвоста оборота 1
}

TEST(BinScan, PlacesPointsByTrueAngle)
{
  std::vector<uint8_t> f;
  put_frame(f, 90.0, 97.0, 1500);   // 8 точек на 90°…97° с шагом 1°
  const auto s = bin_scan(f, ScanOptions{false, 0.0, 360});
  ASSERT_EQ(s.ranges.size(), 360u);
  EXPECT_NEAR(s.angle_min, -M_PI, 1e-6);
  // угол θ → ячейка (θ + 180°) / 1°
  for (int deg = 90; deg <= 97; ++deg) {
    EXPECT_FLOAT_EQ(s.ranges[deg + 180], 1.5f) << deg;
  }
  EXPECT_TRUE(std::isinf(s.ranges[180 + 89]));
  EXPECT_TRUE(std::isinf(s.ranges[180 + 98]));
}

TEST(BinScan, MirrorAndYaw)
{
  std::vector<uint8_t> f;
  put_frame(f, 10.0, 10.0, 2000);   // все точки на 10°
  // зеркально: −10°, плюс поворот 180° → 170°
  const auto s = bin_scan(f, ScanOptions{true, 180.0, 360});
  EXPECT_FLOAT_EQ(s.ranges[170 + 180], 2.0f);
}

TEST(BinScan, ZeroDistanceSkippedAndNearestWins)
{
  std::vector<uint8_t> f;
  put_frame(f, 45.0, 45.0, 0);      // нет измерений
  put_frame(f, 30.0, 30.0, 3000);
  put_frame(f, 30.0, 30.0, 1000);   // та же ячейка, ближе
  const auto s = bin_scan(f, ScanOptions{false, 0.0, 360});
  EXPECT_TRUE(std::isinf(s.ranges[45 + 180]));
  EXPECT_FLOAT_EQ(s.ranges[30 + 180], 1.0f);
}

// ---------- Синхронизация часов ----------

TEST(TimeSync, UsesSampleWithSmallestRtt)
{
  TimeSync ts;
  EXPECT_FALSE(ts.to_pc_ns(0));
  // Истинное смещение: ПК = ESP·1000 + 5 с. ESP отвечает ровно посередине пути.
  const int64_t off = 5'000'000'000;
  // Медленный и несимметричный замер (RTT 40 мс) — оценка смещения хуже
  ts.add(1'000'000'000, 1'040'000'000, (1'000'000'000 + 30'000'000 - off) / 1000);
  // Быстрый замер (RTT 2 мс)
  ts.add(2'000'000'000, 2'002'000'000, (2'000'000'000 + 1'000'000 - off) / 1000);
  const auto t = ts.to_pc_ns(1'000'000);   // ESP: 1 с
  ASSERT_TRUE(t);
  EXPECT_NEAR(static_cast<double>(*t), 1e9 + off, 1000.0);
  EXPECT_EQ(*ts.best_rtt_ns(), 2'000'000);
}

// ---------- Команды и потери ----------

TEST(Command, DiffDriveKinematics)
{
  // 0.5 м/с вперёд, 1 рад/с влево, колея 0.54 м → L = 500 − 270, R = 500 + 270
  const auto w = wheel_speeds_mm(0.5, 1.0, 0.54);
  EXPECT_EQ(w[0], 230);
  EXPECT_EQ(w[1], 770);
  const auto big = wheel_speeds_mm(100.0, 0.0, 0.54);
  EXPECT_EQ(big[0], 32767);
}

TEST(LossCounter, CountsGapsAndWraparound)
{
  LossCounter lc;
  for (uint16_t s : {65533, 65534, 1, 2, 2, 5}) {   // потеряны 65535, 0, 3, 4; дубль 2
    lc.add(s);
  }
  EXPECT_EQ(lc.received(), 5u);
  EXPECT_EQ(lc.lost(), 4u);
}
