// Протокол ESP32 ↔ ПК, версия 2. Одинаков для Wi-Fi (UDP) и USB.
// Зеркальная реализация на стороне ROS: src/esp32_bridge/include/esp32_bridge/protocol.hpp —
// при любом изменении правьте обе.
//
// Пакет (все поля little-endian):
//   [type u8][seq u16][stamp u64][payload ...][crc16 u16]
//   stamp — время отправителя: у ESP32 — микросекунды с загрузки (esp_timer),
//           у ПК — наносекунды его часов (ESP возвращает его в PONG как есть).
//   crc16 — Modbus (0xA001, старт 0xFFFF) по всем байтам до неё.
// Транспорт:
//   UDP — один пакет = одна датаграмма;
//   USB — пакет кодируется COBS и завершается байтом 0x00.
//
// ESP32 → ПК:
//   0x01 ODOM  — 5 × float32: x м, y м, th рад, скорость левого и правого колеса мм/с
//   0x02 SCAN  — rev u16, chunk u8, flags u8 (бит 0 — последний кусок оборота),
//                далее N кадров по 20 байт: угол начала u16 (°×100), угол конца u16,
//                8 дистанций u16 мм (0 — нет измерения). stamp — время первого кадра.
//   0x03 PONG  — u64 stamp из PING как есть (stamp заголовка — время ESP при ответе)
//   0x04 LOG   — текст (UTF-8) — отладочные сообщения прошивки
// ПК → ESP32:
//   0x81 CMD        — int16 скорость левого, int16 правого колеса, мм/с
//   0x82 PING       — без payload (время ПК — в stamp заголовка)
//   0x83 RESET_ODOM — без payload, обнулить одометрию
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace proto
{

constexpr uint8_t ODOM = 0x01;
constexpr uint8_t SCAN = 0x02;
constexpr uint8_t PONG = 0x03;
constexpr uint8_t LOG = 0x04;
constexpr uint8_t CMD = 0x81;
constexpr uint8_t PING = 0x82;
constexpr uint8_t RESET_ODOM = 0x83;

constexpr size_t HEADER_SIZE = 1 + 2 + 8;
constexpr size_t CRC_SIZE = 2;
constexpr size_t MAX_PACKET = 512;
constexpr uint8_t SCAN_FLAG_LAST = 0x01;

inline uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// Собирает пакет в out (не меньше HEADER_SIZE + len + CRC_SIZE байт), возвращает длину.
inline size_t build(uint8_t * out, uint8_t type, uint16_t seq, uint64_t stamp,
                    const uint8_t * payload, size_t len)
{
  out[0] = type;
  memcpy(out + 1, &seq, 2);      // ESP32 — little-endian, копируем как есть
  memcpy(out + 3, &stamp, 8);
  if (len) memcpy(out + HEADER_SIZE, payload, len);
  const uint16_t crc = crc16(out, HEADER_SIZE + len);
  memcpy(out + HEADER_SIZE + len, &crc, 2);
  return HEADER_SIZE + len + CRC_SIZE;
}

// Проверяет CRC. true — пакет цел; type/stamp/payload заполняются.
inline bool parse(const uint8_t * in, size_t n, uint8_t & type, uint64_t & stamp,
                  const uint8_t *& payload, size_t & len)
{
  if (n < HEADER_SIZE + CRC_SIZE) return false;
  uint16_t crc;
  memcpy(&crc, in + n - 2, 2);
  if (crc16(in, n - 2) != crc) return false;
  type = in[0];
  memcpy(&stamp, in + 3, 8);
  payload = in + HEADER_SIZE;
  len = n - HEADER_SIZE - CRC_SIZE;
  return true;
}

// COBS: убирает из данных байт 0x00, чтобы им можно было разделять пакеты в потоке.
// out должен вмещать len + len/254 + 1 байт. Возвращает длину без завершающего 0x00.
inline size_t cobs_encode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t code_pos = 0, o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (in[i] == 0) {
      out[code_pos] = code;
      code_pos = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFF) {
        out[code_pos] = code;
        code_pos = o++;
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return o;
}

// Обратное преобразование. Возвращает длину или 0, если данные битые.
inline size_t cobs_decode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t i = 0, o = 0;
  while (i < len) {
    const uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > len) return 0;
    for (uint8_t k = 1; k < code; ++k) out[o++] = in[i++];
    if (code < 0xFF && i < len) out[o++] = 0;
  }
  return o;
}

}  // namespace proto
