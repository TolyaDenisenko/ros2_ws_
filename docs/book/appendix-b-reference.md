# Приложение B. Справочник команд и неисправностей

## Сборка

```bash
source /opt/ros/kilted/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test --packages-select esp32_bridge && colcon test-result --verbose
./build/esp32_bridge/test_protocol --gtest_filter='BinScan.*'
git submodule update --init
```

## Прошивки

```bash
cd firmware/esp32_bridge && pio run -e wifi -e usb          # ESP32: сборка
pio run -e wifi -t upload                                     # ESP32: Wi-Fi
pio run -e usb -t upload --upload-port /dev/ttyACM0           # ESP32: USB
cd firmware/hoverboard-firmware-hack-FOC
pio run -e VARIANT_USART -t upload                            # плата, ST-Link
```

## Запуск

| Сценарий | Команда |
|---|---|
| Симуляция | `ros2 launch hoverbot_sim sim.launch.py` |
| Симуляция + SLAM | `ros2 launch hoverbot_sim mapping.launch.py` |
| Симуляция + SLAM + Nav2 | `ros2 launch hoverbot_nav sim_navigation.launch.py` |
| Робот + SLAM | `ros2 launch esp32_bridge robot.launch.py transport:=udp host:=<IP>` |
| Робот + Nav2 ⚠ | `ros2 launch hoverbot_nav robot_navigation.launch.py transport:=udp host:=<IP>` |
| Только мост | `ros2 run esp32_bridge esp32_bridge --ros-args -p transport:=serial` |
| Пошаговый запуск железа | `scripts/hardware_bringup.sh <режим>`, режимы `check`, `link`, `map`, `nav` |
| Управление | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| Обнулить одометрию | `ros2 service call /reset_odom std_srvs/srv/Trigger` |
| Сохранить карту | `ros2 run nav2_map_server map_saver_cli -f ~/map` |

## Неисправности

| Симптом | Вероятная причина | Глава |
|---|---|---|
| ROS-команды зависают без вывода | битая общая память Fast DDS: `fastdds shm clean` | 10 |
| `От ESP32 ничего не пришло` | IP, сеть 5 ГГц, не та прошивка | 5.8, 10 |
| `/odom` не меняется | скорость UART плата ↔ ESP32, TX/RX | 2.3 |
| `/scan` нет | пины лидара 32/33 на S3 | 3.4 |
| `imu=0` | провода/пины MPU6050 | 3.4 |
| Робот едет назад / крутится не туда | знаки колёс, `steer` | 2.3, 3.6 |
| Стены в `odom` крутятся при повороте | `lidar_mirror`, `lidar_yaw_deg` | 5.9 |
| Карта не замыкается | калибровка одометрии; второй `map → odom` | 8.7, 10 |
| slam_toolbox молчит | запущен как обычная нода, `unconfigured` | 8.2 |
| Предупреждения TF extrapolation | разный `use_sim_time` | 7.1 |
| Скан в симуляции не доходит до SLAM | нет `gz_frame_id` | 7.3 |
| Nav2-ноды не активны | lifecycle_manager стартовал рано | 9.10 |
| План есть, робот стоит | `TwistStamped` вместо `Twist` | 9.2 |
| Робот в симуляции не останавливается | DiffDrive держит последнюю команду | 7.4 |
| Колёса не останавливаются при обрыве (железо) | **опасно**: проверить `CMD_TIMEOUT_MS`, логику `loop()` | 3.6, 10 |

## Словарь

| Термин | Значение |
|---|---|
| FOC | field-oriented control — векторное управление BLDC-мотором |
| COBS | кодирование, убирающее нули из данных, чтобы `0x00` служил разделителем |
| RTT | round-trip time — время «туда-обратно» |
| TF | система преобразований координат ROS |
| Lifecycle-нода | нода с состояниями unconfigured → inactive → active |
| Costmap | карта стоимости проезда для Nav2 |
| Footprint | контур робота в плоскости |
| Loop closure | замыкание петли — распознавание уже посещённого места |
| QoS | качество обслуживания DDS: reliable/best effort, durability и т. д. |
