# Тележка на шасси гироскутера (ROS 2)

Робот из дешёвых компонентов: шасси гироскутера с прошивкой
[hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC),
плата ESP32-S3, лидар от робота-пылесоса, IMU MPU6050.
Строит карту помещения через slam_toolbox. Есть модель для симуляции в Gazebo.

## Как всё связано

```
 Плата гироскутера ──UART (HB_BAUD)──► ESP32-S3 ◄──UART── лидар пылесоса
 (EFeru FOC, об/мин)                     │  ▲      ◄──I2C── MPU6050
                                         │  │  Wi-Fi (UDP)  или  USB-кабель
                                         ▼  │
                      esp32_bridge (ROS 2): /scan, /odom, TF ◄── /cmd_vel
                                         │
                                         ▼
                              slam_toolbox → /map → RViz
```

Для ESP32 есть **две прошивки из одного кода**: `wifi` — ПК с ROS стоит
отдельно, связь по Wi-Fi; `usb` — ПК (мини-ПК, ноутбук, Raspberry Pi) стоит
на тележке и соединён кабелем. Протокол у них один, нода на ПК тоже одна —
меняется только параметр `transport`.

В симуляции место ESP32 и esp32_bridge занимает Gazebo: топики и кадры
(`/scan`, `/odom`, `/cmd_vel`, `odom → base_link → laser`) те же самые,
поэтому одни и те же настройки SLAM работают и там, и на роботе.

## Шпаргалка

Все команды — из корня рабочего пространства `~/ros2_ws_`. В каждом новом терминале сначала:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash      # на этом ПК: /opt/ros/kilted/setup.bash
source ~/ros2_ws_/install/setup.bash
```

### Сборка и тесты (ПК)

| Что | Команда |
|---|---|
| Собрать всё | `colcon build --symlink-install` |
| Собрать один пакет | `colcon build --symlink-install --packages-select esp32_bridge` |
| Пересобрать с нуля | `rm -rf build install log && colcon build --symlink-install` |
| Все тесты | `colcon test && colcon test-result --verbose` |
| Тесты протокола напрямую | `./build/esp32_bridge/test_protocol` |
| Один тест | `./build/esp32_bridge/test_protocol --gtest_filter='BinScan.*'` |
| Скачать прошивку гироскутера (подмодуль) | `git submodule update --init` |

### Прошивка ESP32-S3 (из папки `firmware/esp32_bridge`)

| Что | Команда |
|---|---|
| Настроить Wi-Fi (один раз) | `cp include/secrets.example.h include/secrets.h` и вписать сеть |
| Собрать обе прошивки | `pio run -e wifi -e usb` |
| Залить Wi-Fi-прошивку | `pio run -e wifi -t upload` |
| Залить USB-прошивку | `pio run -e usb -t upload` |
| Залить на конкретный порт | `pio run -e usb -t upload --upload-port /dev/ttyACM0` |
| Монитор порта (сборка `wifi`, отладка) | `pio device monitor` |
| Очистить сборку | `pio run -t clean` |

Если `pio` не найден: `~/.platformio/penv/bin/pio` (или `pip install platformio`).
В сборке `usb` порт занят протоколом — монитор там не нужен, сообщения прошивки
приходят в лог ноды с префиксом `[esp32]`.

### Симуляция

| Что | Команда |
|---|---|
| Gazebo + RViz + построение карты | `ros2 launch hoverbot_sim mapping.launch.py` |
| То же без окна Gazebo | `ros2 launch hoverbot_sim mapping.launch.py gui:=false` |
| Только Gazebo + RViz | `ros2 launch hoverbot_sim sim.launch.py` |
| Свои параметры SLAM | `ros2 launch hoverbot_sim mapping.launch.py slam_params:=<файл.yaml>` |

### Реальный робот

| Что | Команда |
|---|---|
| Всё сразу, Wi-Fi | `ros2 launch esp32_bridge robot.launch.py transport:=udp host:=<IP ESP32>` |
| Всё сразу, USB | `ros2 launch esp32_bridge robot.launch.py transport:=serial` |
| Другой USB-порт | `ros2 launch esp32_bridge robot.launch.py transport:=serial device:=/dev/ttyACM1` |
| Без SLAM / без RViz | добавить `slam:=false` / `rviz:=false` |
| Только нода связи | `ros2 run esp32_bridge esp32_bridge --ros-args -p transport:=serial` |
| Только SLAM | `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/esp32_bridge/config/slam_param.yaml use_sim_time:=false` |
| Доступ к USB-порту (один раз, перелогиниться) | `sudo usermod -aG dialout $USER` |

### Управление и карта (и в симуляции, и на роботе)

| Что | Команда |
|---|---|
| Управление с клавиатуры | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (`i` вперёд, `j`/`l` поворот, `k` стоп) |
| Сохранить карту | `ros2 run nav2_map_server map_saver_cli -f ~/map` (→ `~/map.pgm` + `~/map.yaml`) |
| Обнулить одометрию (робот) | `ros2 service call /reset_odom std_srvs/srv/Trigger` |
| Twist → TwistStamped | `ros2 run twist_converter twist_to_twist_stamped --ros-args -p input_topic:=/cmd_vel -p output_topic:=/cmd_vel_stamped` |

### Диагностика

| Что | Команда |
|---|---|
| Частота скана / одометрии | `ros2 topic hz /scan`, `ros2 topic hz /odom` |
| Одометрия | `ros2 topic echo /odom --field pose.pose` |
| Где робот на карте | `ros2 run tf2_ros tf2_echo map base_link` |
| Положение лидара | `ros2 run tf2_ros tf2_echo base_link laser` |
| Дерево кадров (PDF) | `ros2 run tf2_tools view_frames` |
| Работает ли SLAM | `ros2 lifecycle get /slam_toolbox` (должно быть `active`) |
| Список топиков / нод | `ros2 topic list`, `ros2 node list` |
| Связь с ESP32 | смотреть строку `Связь: принято …, потеряно …` в логе ноды (раз в 10 с) |
| Команды ROS зависают | `fastdds shm clean`, если не помогло — `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |

### Важные пути

| Путь | Что там |
|---|---|
| `~/ros2_ws_/` | рабочее пространство (корень репозитория) |
| `src/` | исходники ROS-пакетов: `esp32_bridge`, `hoverbot_sim`, `twist_converter` |
| `install/` | собранные пакеты (создаётся `colcon build`, в git не хранится) |
| `build/`, `log/` | промежуточные файлы и логи сборки (в git не хранятся) |
| `~/.ros/log/` | логи запусков нод и launch-файлов |
| `firmware/esp32_bridge/` | прошивка ESP32-S3 (PlatformIO) |
| `firmware/esp32_bridge/PINOUT.md` | схема подключения проводов |
| `firmware/esp32_bridge/include/config.h` | пины, геометрия, тайминги прошивки |
| `firmware/esp32_bridge/include/secrets.h` | Wi-Fi (только локально) |
| `firmware/esp32_bridge/.pio/build/<wifi\|usb>/firmware.bin` | собранная прошивка |
| `firmware/hoverboard-firmware-hack-FOC/` | прошивка платы гироскутера (подмодуль) |
| `src/esp32_bridge/config/slam_param.yaml` | настройки SLAM для робота |
| `src/esp32_bridge/config/slam_localization.yaml` | SLAM в режиме локализации по готовой карте |
| `src/esp32_bridge/config/nav_param.yaml` | настройки навигации Nav2 |
| `src/esp32_bridge/config/rviz.rviz` | вид RViz для робота |
| `src/hoverbot_sim/config/slam_sim.yaml` | настройки SLAM для симуляции (= робот, кроме `use_sim_time`) |
| `src/hoverbot_sim/urdf/hoverbot.urdf.xacro` | модель тележки, все размеры в начале файла |
| `src/hoverbot_sim/worlds/room.sdf` | мир симуляции |
| `/dev/ttyACM0` | ESP32-S3 по USB |

## Где что находится

| Путь | Что это |
|---|---|
| `firmware/esp32_bridge/` | **Прошивка ESP32-S3** (PlatformIO): сборки `wifi` и `usb` |
| `firmware/esp32_bridge/PINOUT.md` | **Схема подключения** ESP32-S3: какой провод на какой пин |
| `firmware/esp32_bridge/include/config.h` | Пины, геометрия робота, тайминги |
| `firmware/esp32_bridge/include/secrets.h` | Имя и пароль Wi-Fi (создать из `secrets.example.h`, в git не попадает) |
| `firmware/esp32_bridge/include/protocol.h` | Протокол ESP32 ↔ ПК (описание формата — в начале файла) |
| `firmware/esp32_bridge/include/transport.h` | Канал связи: UDP или USB, выбирается при сборке |
| `firmware/esp32_bridge/src/main.cpp` | Колёса, одометрия, IMU, лидар |
| `firmware/hoverboard-firmware-hack-FOC/` | Git-подмодуль: прошивка платы гироскутера (STM32). Получить: `git submodule update --init`. Скорость UART в её `Inc/config.h` (`USART2_BAUD`/`USART3_BAUD`, по умолчанию 115200) должна совпадать с `HB_BAUD` в `config.h` ESP32 (сейчас 1000000) |
| `src/esp32_bridge/` | ROS-нода связи с ESP32 (C++), конфиги SLAM/Nav2/RViz для реального робота (`config/`) |
| `src/esp32_bridge/include/esp32_bridge/protocol.hpp`, `src/protocol.cpp` | Тот же протокол на стороне ПК: пакеты, COBS, сборка скана, синхронизация часов |
| `src/esp32_bridge/src/bridge_node.cpp` | Сама нода: UDP или USB, `/scan`, `/odom`, TF, `/cmd_vel`, сервис `/reset_odom` |
| `src/esp32_bridge/launch/robot.launch.py` | Реальный робот: нода + slam_toolbox + RViz |
| `src/esp32_bridge/test/test_protocol.cpp` | Тесты протокола, в т.ч. совместимость с кодом прошивки |
| `src/hoverbot_sim/` | **Симуляция**: модель тележки, мир-комната, launch-файлы |
| `src/hoverbot_sim/urdf/hoverbot.urdf.xacro` | Модель робота. Все размеры — в блоке параметров в начале файла |
| `src/hoverbot_sim/worlds/room.sdf` | Комната 8×6 м с островом посередине (для проверки замыкания петель) |
| `src/hoverbot_sim/config/bridge.yaml` | Какие топики передаются между Gazebo и ROS |
| `src/hoverbot_sim/config/slam_sim.yaml` | Параметры slam_toolbox для симуляции |
| `src/hoverbot_sim/launch/sim.launch.py` | Gazebo + робот + RViz |
| `src/hoverbot_sim/launch/mapping.launch.py` | То же + построение карты |
| `src/twist_converter/` | Преобразование `Twist` → `TwistStamped` (топики — параметры `input_topic`, `output_topic`) |

## Установка (ПК)

ROS 2 (Jazzy или Kilted) + Gazebo Sim. Дополнительно:

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-ros-gz \
                 ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-nav2-map-server \
                 libboost-dev
git submodule update --init     # прошивка гироскутера

cd ~/ros2_ws_
colcon build --symlink-install
source install/setup.bash
```

Тесты протокола:
```bash
colcon test --packages-select esp32_bridge && colcon test-result --verbose
```

Для USB-сборки пользователю нужен доступ к порту (один раз, затем перелогиниться):
```bash
sudo usermod -aG dialout $USER
```

## Прошивка ESP32-S3

Нужен [PlatformIO](https://platformio.org/) (`pip install platformio` или расширение VS Code).

```bash
cd firmware/esp32_bridge
cp include/secrets.example.h include/secrets.h   # вписать свою сеть (для wifi)
pio run -e wifi -t upload     # Wi-Fi
pio run -e usb  -t upload     # USB
```

- Подключение — по схеме в `PINOUT.md` (MPU6050: SDA → GPIO 8, SCL → GPIO 9).
- Перед прошивкой проверьте пин лидара в `include/config.h`. Он взят из старой прошивки,
  но для ESP32-S3 сомнителен: GPIO 26–37 на многих модулях
  заняты флеш-памятью/PSRAM.
- USB-сборка работает через порт платы с надписью **USB** (встроенный USB ESP32-S3),
  на ПК он появится как `/dev/ttyACM0`.
- Wi-Fi-сборка слушает UDP-порт 8888 и отвечает тому ПК, который шлёт ей пакеты.
  IP платы нужно знать (посмотреть в роутере или закрепить за ней адрес).
- При обрыве связи с ПК дольше 300 мс колёса останавливаются.

## Запуск

**Симуляция с построением карты** (Gazebo + RViz + slam_toolbox):
```bash
ros2 launch hoverbot_sim mapping.launch.py
# в другом терминале — управление с клавиатуры:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# сохранить карту:
ros2 run nav2_map_server map_saver_cli -f ~/map
```
Аргументы: `gui:=false` — без окна Gazebo; `slam_params:=<файл.yaml>` — свои параметры SLAM.

Только симуляция, без SLAM: `ros2 launch hoverbot_sim sim.launch.py`.

**Реальный робот** (нода + SLAM + RViz):
```bash
ros2 launch esp32_bridge robot.launch.py transport:=udp host:=<IP ESP32>   # Wi-Fi
ros2 launch esp32_bridge robot.launch.py transport:=serial                 # USB
```
Аргументы: `slam:=false` — без построения карты; `rviz:=false` — без RViz;
`device:=/dev/ttyACM1` — другой порт.

Только нода: `ros2 run esp32_bridge esp32_bridge --ros-args -p transport:=serial`.

Обнулить одометрию: `ros2 service call /reset_odom std_srvs/srv/Trigger`.

Параметры ноды `esp32_bridge` (`--ros-args -p имя:=значение`):

| Параметр | По умолчанию | Что это |
|---|---|---|
| `transport` | udp | `udp` (Wi-Fi) или `serial` (USB) |
| `host`, `port` | 192.168.0.105, 8888 | Адрес ESP32 (udp) |
| `device`, `baud` | /dev/ttyACM0, 921600 | Порт ESP32 (serial; у встроенного USB S3 скорость не важна) |
| `wheel_base` | 0.54 | Колея, м — должна совпадать с `WHEEL_BASE` в прошивке |
| `cmd_timeout` | 0.5 | Сколько секунд повторять последнюю команду (20 Гц) после последнего `/cmd_vel` |
| `scan_bins` | 360 | Угловых ячеек в `/scan` (360 — шаг 1°) |
| `lidar_xyz`, `lidar_rpy_deg` | [0.10, 0, 0.17], [0, 0, 0] | Положение и ориентация лидара от оси колёс (TF `base_link→laser`) |
| `lidar_mirror`, `lidar_yaw_deg` | true, 180 | Отражение и поворот облака под монтаж лидара |
| `range_min`, `range_max` | 0.05, 16.0 | Диапазон дальностей в `/scan`, м |
| `frame_id`, `odom_frame`, `base_frame` | laser, odom, base_link | Имена кадров |
| `publish_map_odom_tf` | false | Нулевой TF `map→odom` — только для RViz **без** SLAM. Вместе со SLAM не включать: он перебивает поправку SLAM, и карта не замыкается |

Раз в 10 с нода пишет в лог статистику связи: сколько пакетов принято и
потеряно, ошибки CRC, неполные обороты лидара, время отклика ESP32.
Сообщения самой прошивки появляются в логе с префиксом `[esp32]`.

## Протокол ESP32 ↔ ПК

Полное описание — в начале `firmware/esp32_bridge/include/protocol.h`. Кратко:

- Каждый пакет: тип, номер (по номерам нода считает потери), **время ESP32 в
  микросекундах**, данные, CRC16.
- Нода раз в 0,5 с шлёт PING и по ответам оценивает разницу часов ПК и ESP32.
  Поэтому метки времени `/scan` и `/odom` — это момент измерения, а не момент
  прихода пакета, и задержка Wi-Fi не попадает в SLAM.
- Скан идёт кусками по 8 кадров лидара по мере вращения. Нода собирает оборот и
  раскладывает точки по угловым ячейкам по их настоящим углам — потерянный кусок
  даёт пробел в скане, а не сдвиг углов.
- UDP: пакет = датаграмма; свежие данные не ждут переотправки потерянных, как в TCP.
- USB: пакеты кодируются COBS и разделяются байтом 0x00.

При изменении формата правьте обе стороны: `protocol.h` (прошивка) и
`protocol.hpp`/`protocol.cpp` (нода). Тесты ноды подключают `protocol.h` прошивки
и проверяют, что пакеты одной стороны разбираются другой.

## Геометрия робота

Используется в трёх местах, значения должны совпадать:

| Величина | Значение | Прошивка (`config.h`) | Параметр `esp32_bridge` | `hoverbot.urdf.xacro` |
|---|---|---|---|---|
| Колея (центр шины ↔ центр шины) | 0.54 м (снаружи 0.60, изнутри 0.48) | `WHEEL_BASE` | `wheel_base` | `wheel_separation` |
| Диаметр колеса | 0.16 м | `WHEEL_DIAMETER` | — | `wheel_radius` (×½) |
| Положение лидара (от оси колёс) | x +0.10 (вперёд), z 0.17 (0.25 м от пола) | — | `lidar_xyz` | `lidar_x`, `lidar_height` |

Перёд робота — со стороны кастера, площадка 50×30 см идёт от оси вперёд.

## Проверка одометрии на реальном роботе

1. `ros2 service call /reset_odom std_srvs/srv/Trigger`, проехать ровно 1 м по
   рулетке — в `ros2 topic echo /odom` должно быть x ≈ 1.0. Если нет — поправить
   `WHEEL_DIAMETER` в `config.h`.
2. Развернуть робота на месте на 360° — `th` должен вернуться к ≈ 0.
   Если нет — поправить `WHEEL_BASE` (и параметр `wheel_base`).
3. RViz, Fixed Frame = `odom`, `/scan` с Decay Time 10 с, медленно
   покрутить робота на месте: стены должны стоять на месте. Если крутятся
   в обратную сторону — поменять `lidar_mirror` / `lidar_yaw_deg`.

Пока шаг 3 не проходит, SLAM не сможет замкнуть контур карты.

## Если ROS-команды зависают

Если `ros2 topic echo`, `ros2 node list` или launch-файлы зависают без вывода
(обычно после принудительно убитых процессов), это битые сегменты общей памяти
Fast DDS. Помогает:

```bash
fastdds shm clean
```

Если не помогло — обмен только по UDP (в каждом терминале):

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```
