#include "esp32_bridge/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace esp32_bridge
{

namespace
{

// Чтение/запись little-endian независимо от платформы и выравнивания
uint16_t rd16(const uint8_t * p) {return static_cast<uint16_t>(p[0] | (p[1] << 8));}

uint64_t rd64(const uint8_t * p)
{
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8) | p[i];
  }
  return v;
}

float rdf(const uint8_t * p)
{
  const uint32_t bits = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

void wr16(std::vector<uint8_t> & v, uint16_t x)
{
  v.push_back(x & 0xFF);
  v.push_back(x >> 8);
}

void wr64(std::vector<uint8_t> & v, uint64_t x)
{
  for (int i = 0; i < 8; ++i) {
    v.push_back((x >> (8 * i)) & 0xFF);
  }
}

}  // namespace

uint16_t crc16(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

std::vector<uint8_t> build_packet(
  uint8_t type, uint16_t seq, uint64_t stamp, const uint8_t * payload, std::size_t len)
{
  std::vector<uint8_t> p;
  p.reserve(kHeaderSize + len + kCrcSize);
  p.push_back(type);
  wr16(p, seq);
  wr64(p, stamp);
  p.insert(p.end(), payload, payload + len);
  wr16(p, crc16(p.data(), p.size()));
  return p;
}

std::optional<Packet> parse_packet(const uint8_t * data, std::size_t len)
{
  if (len < kHeaderSize + kCrcSize) {
    return std::nullopt;
  }
  if (crc16(data, len - kCrcSize) != rd16(data + len - kCrcSize)) {
    return std::nullopt;
  }
  return Packet{data[0], rd16(data + 1), rd64(data + 3),
    std::vector<uint8_t>(data + kHeaderSize, data + len - kCrcSize)};
}

// ---------- COBS ----------

std::vector<uint8_t> cobs_encode(const std::vector<uint8_t> & in)
{
  std::vector<uint8_t> out(1);
  std::size_t code_pos = 0;
  uint8_t code = 1;
  for (uint8_t b : in) {
    if (b == 0) {
      out[code_pos] = code;
      code_pos = out.size();
      out.push_back(0);
      code = 1;
    } else {
      out.push_back(b);
      if (++code == 0xFF) {
        out[code_pos] = code;
        code_pos = out.size();
        out.push_back(0);
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return out;
}

std::optional<std::vector<uint8_t>> cobs_decode(const uint8_t * in, std::size_t len)
{
  std::vector<uint8_t> out;
  std::size_t i = 0;
  while (i < len) {
    const uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > len) {
      return std::nullopt;
    }
    for (uint8_t k = 1; k < code; ++k) {
      out.push_back(in[i++]);
    }
    if (code < 0xFF && i < len) {
      out.push_back(0);
    }
  }
  return out;
}

std::vector<std::vector<uint8_t>> CobsStream::feed(const uint8_t * data, std::size_t len)
{
  std::vector<std::vector<uint8_t>> packets;
  for (std::size_t i = 0; i < len; ++i) {
    if (data[i] != 0) {
      if (buf_.size() < 4096) {   // защита от мусора без разделителей
        buf_.push_back(data[i]);
      }
      continue;
    }
    if (!buf_.empty()) {
      if (auto p = cobs_decode(buf_.data(), buf_.size())) {
        packets.push_back(std::move(*p));
      }
    }
    buf_.clear();
  }
  return packets;
}

// ---------- Одометрия ----------

std::optional<OdomSample> parse_odom(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 5 * sizeof(float)) {
    return std::nullopt;
  }
  const uint8_t * p = payload.data();
  return OdomSample{rdf(p), rdf(p + 4), rdf(p + 8), rdf(p + 12), rdf(p + 16)};
}

// ---------- Скан ----------

std::optional<ScanChunk> parse_scan_chunk(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 4 || (payload.size() - 4) % kFrameSize != 0) {
    return std::nullopt;
  }
  return ScanChunk{rd16(payload.data()), payload[2], (payload[3] & 0x01) != 0,
    std::vector<uint8_t>(payload.begin() + 4, payload.end())};
}

std::optional<Revolution> ScanAssembler::add(const ScanChunk & chunk, uint64_t stamp_us)
{
  // Пришёл кусок нового оборота, а конец старого потерян — старый выбрасываем
  if (!rev_ || chunk.rev != *rev_) {
    rev_ = chunk.rev;
    next_index_ = 0;
    cur_ = Revolution{{}, stamp_us, stamp_us, true};
  }
  if (chunk.index != next_index_) {
    cur_.complete = false;   // кусок(и) потерялся
  }
  next_index_ = static_cast<uint8_t>(chunk.index + 1);
  cur_.frames.insert(cur_.frames.end(), chunk.frames.begin(), chunk.frames.end());
  cur_.last_stamp_us = stamp_us;

  if (!chunk.last) {
    return std::nullopt;
  }
  Revolution done = std::move(cur_);
  rev_.reset();
  cur_ = Revolution{};
  return done;
}

Scan bin_scan(const std::vector<uint8_t> & frames, const ScanOptions & opt)
{
  const std::size_t bins = std::max<std::size_t>(opt.bins, 1);
  const double inc_deg = 360.0 / static_cast<double>(bins);

  Scan scan;
  scan.angle_increment = static_cast<float>(inc_deg * M_PI / 180.0);
  scan.angle_min = static_cast<float>(-M_PI);
  scan.angle_max = scan.angle_min + scan.angle_increment * static_cast<float>(bins - 1);
  scan.ranges.assign(bins, std::numeric_limits<float>::infinity());
  scan.intensities.assign(bins, 0.0f);

  const std::size_t n = frames.size() / kFrameSize;
  for (std::size_t i = 0; i < n; ++i) {
    const uint8_t * f = frames.data() + i * kFrameSize;
    const double s = rd16(f) / 100.0;
    double e = rd16(f + 2) / 100.0;
    if (e < s) {
      e += 360.0;
    }
    for (std::size_t j = 0; j < kSamplesPerFrame; ++j) {
      const uint16_t d = rd16(f + 4 + 2 * j);
      if (d == 0) {
        continue;   // нет измерения
      }
      double a = s + (e - s) * static_cast<double>(j) / (kSamplesPerFrame - 1);
      if (opt.mirror) {
        a = -a;     // лидар вращается в обратную сторону / смонтирован зеркально
      }
      a += opt.yaw_deg;
      a = std::fmod(a + 180.0, 360.0);   // → [0, 360) от угла −180°
      if (a < 0) {
        a += 360.0;
      }
      const std::size_t bin = static_cast<std::size_t>(a / inc_deg) % bins;
      const float r = d / 1000.0f;
      if (r < scan.ranges[bin]) {
        scan.ranges[bin] = r;
        scan.intensities[bin] = 1.0f;
      }
    }
  }
  return scan;
}

// ---------- Синхронизация часов ----------

void TimeSync::add(int64_t pc_send_ns, int64_t pc_recv_ns, uint64_t esp_us)
{
  const int64_t rtt = pc_recv_ns - pc_send_ns;
  if (rtt < 0) {
    return;
  }
  // ESP ответил примерно посередине пути туда-обратно
  const int64_t offset = pc_send_ns + rtt / 2 - static_cast<int64_t>(esp_us) * 1000;
  samples_.push_back({offset, rtt});
  while (samples_.size() > window_) {
    samples_.pop_front();
  }
}

std::optional<int64_t> TimeSync::to_pc_ns(uint64_t esp_us) const
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  const auto best = std::min_element(samples_.begin(), samples_.end(),
      [](const Sample & a, const Sample & b) {return a.rtt_ns < b.rtt_ns;});
  return static_cast<int64_t>(esp_us) * 1000 + best->offset_ns;
}

std::optional<int64_t> TimeSync::best_rtt_ns() const
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  return std::min_element(samples_.begin(), samples_.end(),
           [](const Sample & a, const Sample & b) {return a.rtt_ns < b.rtt_ns;})->rtt_ns;
}

// ---------- Команды ----------

std::array<int16_t, 2> wheel_speeds_mm(double linear_x, double angular_z, double wheel_base_m)
{
  const double v_mm = linear_x * 1000.0;
  const double half_base_mm = wheel_base_m * 1000.0 / 2.0;
  auto clamp16 = [](double v) {
      return static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
    };
  return {clamp16(v_mm - angular_z * half_base_mm), clamp16(v_mm + angular_z * half_base_mm)};
}

std::vector<uint8_t> encode_command(int16_t left, int16_t right)
{
  std::vector<uint8_t> v;
  wr16(v, static_cast<uint16_t>(left));
  wr16(v, static_cast<uint16_t>(right));
  return v;
}

void LossCounter::add(uint16_t seq)
{
  ++received_;
  if (last_) {
    const uint16_t gap = static_cast<uint16_t>(seq - *last_);
    if (gap == 0) {
      --received_;          // дубликат
      return;
    }
    if (gap < 1000) {
      lost_ += gap - 1;
    }
    // gap ≥ 1000 — ESP перезагрузилась или пакеты пришли не по порядку; не считаем
  }
  last_ = seq;
}

}  // namespace esp32_bridge
