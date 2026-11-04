// Настройки железа и робота. Значения геометрии должны совпадать с
// параметром wheel_base ноды esp32_bridge и с hoverbot_sim/urdf/hoverbot.urdf.xacro.
#pragma once

#include <math.h>
#include <stdint.h>

/* ---------- Плата гироскутера (EFeru hoverboard-firmware-hack-FOC) ---------- */
#define HB_SERIAL Serial2
constexpr uint32_t HB_BAUD = 1000000;   // = USART*_BAUD в Inc/config.h прошивки гироскутера
constexpr int HB_RX_PIN = 18;
constexpr int HB_TX_PIN = 17;

/* ---------- Лидар от пылесоса ---------- */
// TODO: проверить подключение (см. PINOUT.md)! Пины взяты из старой прошивки, но для ESP32-S3
// они сомнительны: GPIO 26–32 у модулей S3 заняты флеш-памятью, GPIO 33–37 —
// PSRAM на модулях с 8 МБ PSRAM (…R8), а GPIO 22 у S3 вообще нет (см. IMU ниже).
// Если лидар/IMU не отвечают — перекиньте на свободные GPIO (например GPIO 4/5, свободные — в PINOUT.md).
#define LIDAR_SERIAL Serial1
constexpr uint32_t LIDAR_BAUD = 115200;
constexpr int LIDAR_RX_PIN = 32;
constexpr int LIDAR_TX_PIN = 33;

/* ---------- MPU6050 ---------- */
// I2C-пины ESP32-S3 по умолчанию. Раньше было SDA=21, SCL=22 — GPIO 22 у S3 нет,
// поэтому IMU не работал. Схема подключения — в PINOUT.md.
constexpr int IMU_SDA_PIN = 8;
constexpr int IMU_SCL_PIN = 9;
constexpr float YAW_KP = 0.8f;    // П-регулятор удержания курса при езде прямо

/* ---------- Геометрия (замеры) ---------- */
constexpr float WHEEL_BASE = 0.54f;      // колея по центрам шин, м: (0.60 снаружи + 0.48 изнутри) / 2
constexpr float WHEEL_DIAMETER = 0.16f;  // диаметр колеса по шине, м
// Телеметрия гироскутера speedR_meas/speedL_meas — об/мин мотор-колеса
constexpr float RPM_TO_MPS = (float)M_PI * WHEEL_DIAMETER / 60.0f;

/* ---------- Тайминги ---------- */
constexpr uint32_t CMD_TIMEOUT_MS = 300;   // нет команд дольше — колёса стоп
constexpr uint32_t HB_CMD_PERIOD_MS = 50;  // команда гироскутеру, 20 Гц
constexpr uint32_t ODOM_PERIOD_MS = 20;    // одометрия на ПК, 50 Гц

/* ---------- Связь ---------- */
constexpr uint16_t UDP_PORT = 8888;        // wifi: ESP слушает этот порт и отвечает на адрес ПК
constexpr uint32_t USB_BAUD = 921600;      // usb: у встроенного USB S3 фактически не важна
