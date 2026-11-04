// Канал связи с ПК. Реализация выбирается при сборке (platformio.ini):
//   -DTRANSPORT_WIFI — UDP по Wi-Fi;
//   -DTRANSPORT_USB  — USB (Serial), пакеты в COBS с разделителем 0x00.
// Оба варианта отдают наверх уже «сырые» пакеты протокола (protocol.h),
// так что основной код прошивки от транспорта не зависит.
#pragma once

#include <Arduino.h>

#include "config.h"
#include "protocol.h"

#if defined(TRANSPORT_WIFI) == defined(TRANSPORT_USB)
#error "Задайте ровно один из флагов: -DTRANSPORT_WIFI или -DTRANSPORT_USB"
#endif

#ifdef TRANSPORT_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"
#endif

class Transport
{
public:
  void begin()
  {
    lock_ = xSemaphoreCreateMutex();
#ifdef TRANSPORT_WIFI
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // без энергосбережения модема — стабильная задержка
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    udp_.begin(UDP_PORT);
#else
    Serial.setRxBufferSize(1024);
    Serial.begin(USB_BAUD);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);      // ПК не читает порт — не блокировать прошивку
#endif
#endif
  }

  // Принимает один пакет, если он есть. Возвращает длину (0 — ничего нет).
  size_t poll(uint8_t * out, size_t max)
  {
#ifdef TRANSPORT_WIFI
    const int n = udp_.parsePacket();
    if (n <= 0) return 0;
    if ((size_t)n > max) {
      udp_.clear();
      return 0;
    }
    candidate_ip_ = udp_.remoteIP();
    candidate_port_ = udp_.remotePort();
    return udp_.read(out, n);
#else
    while (Serial.available()) {
      const uint8_t b = Serial.read();
      if (b != 0) {
        if (rx_len_ < sizeof(rx_)) rx_[rx_len_++] = b;
        else overflow_ = true;
        continue;
      }
      // 0x00 — конец пакета
      size_t n = 0;
      if (!overflow_ && rx_len_ > 0 && rx_len_ <= max) n = proto::cobs_decode(rx_, rx_len_, out);
      rx_len_ = 0;
      overflow_ = false;
      if (n) return n;
    }
    return 0;
#endif
  }

  // Вызывается после проверки CRC: отвечать будем тому, кто прислал корректный пакет.
  void accept_sender()
  {
#ifdef TRANSPORT_WIFI
    peer_ip_ = candidate_ip_;
    peer_port_ = candidate_port_;
    has_peer_ = true;
#endif
  }

  // Потокобезопасная отправка (зовётся из loop() и из задачи лидара).
  void send(const uint8_t * pkt, size_t len)
  {
    xSemaphoreTake(lock_, portMAX_DELAY);
#ifdef TRANSPORT_WIFI
    if (has_peer_ && WiFi.status() == WL_CONNECTED) {
      udp_.beginPacket(peer_ip_, peer_port_);
      udp_.write(pkt, len);
      udp_.endPacket();
    }
#else
    const size_t n = proto::cobs_encode(pkt, len, tx_);
    tx_[n] = 0;
    if (Serial.availableForWrite() >= (int)(n + 1)) {   // нет места — пакет пропускаем, не ждём
      Serial.write(tx_, n + 1);
    }
#endif
    xSemaphoreGive(lock_);
  }

  bool link_up() const
  {
#ifdef TRANSPORT_WIFI
    return WiFi.status() == WL_CONNECTED;
#else
    return true;
#endif
  }

private:
  SemaphoreHandle_t lock_ = nullptr;
#ifdef TRANSPORT_WIFI
  WiFiUDP udp_;
  IPAddress candidate_ip_, peer_ip_;
  uint16_t candidate_port_ = 0, peer_port_ = 0;
  bool has_peer_ = false;
#else
  uint8_t rx_[proto::MAX_PACKET + proto::MAX_PACKET / 254 + 2];
  size_t rx_len_ = 0;
  bool overflow_ = false;
  uint8_t tx_[proto::MAX_PACKET + proto::MAX_PACKET / 254 + 2];
#endif
};
