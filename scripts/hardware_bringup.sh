#!/usr/bin/env bash
# Запуск на РЕАЛЬНОМ железе (ESP32-S3 + плата гироскутера) по данным прошивок.
#
#   scripts/hardware_bringup.sh check                     # сверка прошивок и ROS-параметров, железо не нужно
#   scripts/hardware_bringup.sh link  [udp HOST|serial DEV] # только мост: проверка /odom /scan /imu, колёса НЕ крутятся
#   scripts/hardware_bringup.sh map   [udp HOST|serial DEV] # мост + SLAM + RViz (ручное управление)
#   scripts/hardware_bringup.sh nav   [udp HOST|serial DEV] # мост + SLAM + Nav2 (⚠ робот поедет сам)
# По умолчанию: udp 192.168.0.105.  Переменные: RVIZ=false  OCTOMAP=false

set -u
WS="$(cd "$(dirname "$0")/.." && pwd)"
FW="$WS/firmware/esp32_bridge"
FOC="$WS/firmware/hoverboard-firmware-hack-FOC"
CFG="$FW/include/config.h"
fail=0
ok()   { echo "  [ OK ] $*"; }
warn() { echo "  [WARN] $*"; }
bad()  { echo "  [FAIL] $*"; fail=1; }

# значение constexpr/#define из config.h прошивки ESP32
cfgval() { sed -nE "s/^[[:space:]]*(constexpr[^=]*|#define[[:space:]]+)$1[[:space:]]*=?[[:space:]]*([0-9.]+).*/\2/p" "$CFG" | head -1; }
close()  { awk -v a="$1" -v b="$2" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<1e-6)}'; }

check() {
  echo "== Сверка прошивок и ROS =="
  [ -f "$CFG" ] || { bad "нет $CFG"; return; }
  local base dia baud
  base=$(cfgval WHEEL_BASE | tr -d f); dia=$(cfgval WHEEL_DIAMETER | tr -d f); baud=$(cfgval HB_BAUD)
  echo "  ESP32: WHEEL_BASE=$base м, WHEEL_DIAMETER=$dia м, HB_BAUD=$baud"

  # колея: прошивка ↔ параметр ноды ↔ xacro
  local nb xb xr
  nb=$(grep -oE 'declare_parameter\("wheel_base", *[0-9.]+' "$WS/src/esp32_bridge/src/bridge_node.cpp" | grep -oE '[0-9.]+$')
  xb=$(grep -oE 'name="wheel_separation" value="[0-9.]+' "$WS/src/hoverbot_sim/urdf/hoverbot.urdf.xacro" | grep -oE '[0-9.]+$')
  xr=$(grep -oE 'name="wheel_radius" +value="[0-9.]+' "$WS/src/hoverbot_sim/urdf/hoverbot.urdf.xacro" | grep -oE '[0-9.]+$')
  close "$base" "${nb:-0}" && ok "колея: прошивка = параметр wheel_base ($nb)" || bad "колея: прошивка $base ≠ wheel_base ноды $nb"
  close "$base" "${xb:-0}" && ok "колея: прошивка = xacro ($xb)"               || bad "колея: прошивка $base ≠ xacro $xb"
  close "$dia" "$(awk -v r="${xr:-0}" 'BEGIN{print 2*r}')" && ok "диаметр: прошивка = 2×wheel_radius ($xr)" || bad "диаметр: прошивка $dia ≠ 2×$xr"

  # скорость UART: ESP32 ↔ прошивка гироскутера (EFeru FOC)
  local foc_baud
  foc_baud=$(grep -hoE 'USART[23]_BAUD=?[ ]*[0-9]+' "$FOC/platformio.ini" 2>/dev/null | grep -oE '[0-9]+$' | head -1)
  [ -n "$foc_baud" ] || foc_baud=$(sed -nE 's/^[[:space:]]*#define[[:space:]]+USART2_BAUD[[:space:]]+([0-9]+).*/\1/p' "$FOC/Inc/config.h" | head -1)
  if [ "$foc_baud" = "$baud" ]; then ok "UART гироскутера = $foc_baud бод"
  else bad "UART: ESP32 HB_BAUD=$baud, гироскутер USART*_BAUD=${foc_baud:-?} → добавьте -DUSART2_BAUD=$baud -DUSART3_BAUD=$baud в env VARIANT_USART ($FOC/platformio.ini) и перепрошейте через ST-Link"; fi
  echo "  FOC: прошивать надо env VARIANT_USART:  (cd firmware/hoverboard-firmware-hack-FOC && pio run -e VARIANT_USART -t upload)"

  # тайминги: ESP останавливает колёса через CMD_TIMEOUT_MS, мост шлёт 20 Гц
  local t; t=$(cfgval CMD_TIMEOUT_MS)
  [ "${t:-0}" -ge 100 ] && ok "CMD_TIMEOUT_MS=$t мс (мост шлёт /cmd_vel 20 Гц)" || warn "CMD_TIMEOUT_MS=$t слишком мал"

  # пины: GPIO 26–37 на ESP32-S3 — флеш/PSRAM
  for p in LIDAR_RX_PIN LIDAR_TX_PIN HB_RX_PIN HB_TX_PIN IMU_SDA_PIN IMU_SCL_PIN; do
    v=$(cfgval "$p")
    if [ "${v:-0}" -ge 26 ] && [ "$v" -le 37 ]; then warn "$p=$v — GPIO 26–37 на ESP32-S3 заняты флеш/PSRAM (см. PINOUT.md)"; fi
  done
  [ -f "$FW/include/secrets.h" ] && ok "secrets.h есть" || warn "нет include/secrets.h (нужен для wifi-прошивки)"
  # дублирующий юнит: slam-параметры
  diff -q <(grep -vE "use_sim_time|^\s*#" "$WS/src/hoverbot_sim/config/slam_sim.yaml") \
          <(grep -vE "use_sim_time|^\s*#" "$WS/src/esp32_bridge/config/slam_param.yaml") >/dev/null \
    && ok "slam_sim.yaml = slam_param.yaml" || warn "slam_sim.yaml и slam_param.yaml разошлись"
  echo; [ $fail = 0 ] && echo "Сверка пройдена." || echo "Есть ошибки — исправьте до запуска на железе."
}

ros_env() {
  source /opt/ros/kilted/setup.bash
  [ -f "$WS/install/setup.bash" ] || { echo "Сначала: colcon build --symlink-install"; exit 1; }
  source "$WS/install/setup.bash"
  fastdds shm clean >/dev/null 2>&1 || true
}

link_args() {   # $1=udp|serial $2=host|device
  case "${1:-udp}" in
    udp)    echo "transport:=udp host:=${2:-192.168.0.105}" ;;
    serial) echo "transport:=serial device:=${2:-/dev/ttyACM0}" ;;
    *) echo "транспорт: udp|serial" >&2; exit 2 ;;
  esac
}

hz_check() {
  for t in /odom /scan /imu; do
    r=$(timeout -s KILL 8 ros2 topic hz "$t" --window 20 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
    [ -n "$r" ] && echo "  $t: $r Гц" || echo "  $t: НЕТ данных"
  done
  timeout -s KILL 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -m1 "Translation" >/dev/null \
    && echo "  TF odom→base_link: есть" || echo "  TF odom→base_link: НЕТ"
}

mode="${1:-check}"; shift || true
case "$mode" in
  check) check; exit $fail ;;
  link)
    check; [ $fail = 0 ] || exit 1
    ros_env
    # shellcheck disable=SC2046
    ros2 launch esp32_bridge robot.launch.py $(link_args "$@") slam:=false rviz:=false &
    LP=$!; trap 'kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null' EXIT
    echo "Жду 8 с, проверяю потоки (колёса не командуются; крутите их рукой — /odom должен меняться)…"; sleep 8
    hz_check
    echo "Ctrl+C — выход."; wait $LP ;;
  map)
    check; [ $fail = 0 ] || exit 1
    ros_env
    echo "Управление: ros2 run teleop_twist_keyboard teleop_twist_keyboard  (первый выезд — на подставке!)"
    # shellcheck disable=SC2046
    ros2 launch esp32_bridge robot.launch.py $(link_args "$@") rviz:="${RVIZ:-true}" ;;
  nav)
    check; [ $fail = 0 ] || exit 1
    ros_env
    echo "⚠ Робот поедет сам. Рука на выключателе. Нажмите Enter для старта…"; read -r _
    # shellcheck disable=SC2046
    ros2 launch hoverbot_nav robot_navigation.launch.py $(link_args "$@") rviz:="${RVIZ:-true}" octomap:="${OCTOMAP:-true}" ;;
  *) sed -n 2,10p "$0"; exit 2 ;;
esac
