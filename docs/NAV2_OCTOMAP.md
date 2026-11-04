# Навигация hoverbot: Nav2, costmap и octomap

> Руководство по пакету `hoverbot_nav`: как всё устроено, как собрать, настроить
> и запустить — в симуляции и на реальном железе. Чек-лист состояния работ —
> в [CHECKLIST.md](CHECKLIST.md).

## 1. Идея в одной картинке

Робот уже умел строить карту (slam_toolbox). Теперь он умеет *ехать* в точку
на этой карте, объезжая препятствия. Для этого поверх SLAM работают три слоя:

```
                       ┌────────── slam_toolbox ──────────┐
   /scan ─────────────►│  /map  (OccupancyGrid)           │
   TF odom→base_link ─►│  TF map→odom                     │
                       └───────────────┬──────────────────┘
                                       │ /map
  ┌─────────────────────────── Nav2 ───┼───────────────────────────────┐
  │                                    ▼                               │
  │  global_costmap ──► planner_server ──► smoother_server             │
  │  (static + obstacle + inflation)        (путь)                     │
  │                                           │                        │
  │  local_costmap ───► controller_server ◄───┘                        │
  │  (obstacle + inflation,   (Regulated Pure Pursuit)                 │
  │   окно 4×4 м вокруг робота)      │ /cmd_vel_nav                    │
  │                                  ▼                                 │
  │                         velocity_smoother ──► /cmd_vel ──► колёса  │
  │  bt_navigator (дерево поведения) + behavior_server (spin, backup)  │
  └────────────────────────────────────────────────────────────────────┘

  /scan ─► scan_to_cloud ─► /scan_cloud ─► octomap_server ─► /octomap_binary  (3D)
                                                         └─► /projected_map   (2D-проекция)
```

Ключевые решения:

- **`map_server` и `amcl` не запускаются.** Карту и TF `map→odom` даёт
  slam_toolbox — два источника `map→odom` ломают loop closure.
- **Nav2 публикует `Twist`, а не `TwistStamped`.** В Kilted по умолчанию
  `TwistStamped`; мост `esp32_bridge` и Gazebo DiffDrive ждут `Twist`,
  поэтому в параметрах стоит `enable_stamped_cmd_vel: false`.
- **Цепочка скоростей:** контроллер → `/cmd_vel_nav` → `velocity_smoother` →
  `/cmd_vel`. Сглаживатель ограничивает ускорения, чтобы не буксовать
  тяжёлыми мотор-колёсами.
- **Один пакет на симуляцию и железо.** Разница — только `use_sim_time`.

## 2. Что внутри пакета

```
src/hoverbot_nav/
├── config/nav2_params.yaml        # параметры всех нод Nav2
├── launch/
│   ├── navigation.launch.py       # Nav2 + octomap поверх уже идущего SLAM
│   ├── sim_navigation.launch.py   # симуляция целиком: Gazebo+SLAM+Nav2
│   └── robot_navigation.launch.py # реальный робот целиком: мост+SLAM+Nav2
└── src/scan_to_cloud.cpp          # LaserScan → PointCloud2 для octomap
```

### Ноды

| Нода | Пакет | Роль | Ключевые топики |
|---|---|---|---|
| `controller_server` | nav2_controller | Ведёт по пути, держит **local costmap** | `/cmd_vel_nav` (выход) |
| `planner_server` | nav2_planner | Строит путь по **global costmap** (NavFn, A\*) | `/plan` |
| `smoother_server` | nav2_smoother | Сглаживает путь | — |
| `behavior_server` | nav2_behaviors | Восстановление: поворот, откат, ожидание | `/cmd_vel_nav` |
| `bt_navigator` | nav2_bt_navigator | Дерево поведения, action `/navigate_to_pose` | — |
| `velocity_smoother` | nav2_velocity_smoother | Ограничение ускорений | `/cmd_vel_nav` → `/cmd_vel` |
| `lifecycle_manager_navigation` | nav2_lifecycle_manager | Конфигурирует и активирует всё перечисленное | — |
| `scan_to_cloud` | hoverbot_nav | `/scan` → `/scan_cloud` (PointCloud2 в кадре лидара) | — |
| `octomap_server` | octomap_server | Вокселы 5 см, лучевая трассировка из позиции лидара | `/octomap_binary`, `/projected_map` |

Все ноды Nav2 — **lifecycle**. Состояние проверяется так:
`ros2 lifecycle get /planner_server` → `active [3]`.

### Costmap'ы

| | local | global |
|---|---|---|
| Кадр | `odom`, окно 4×4 м едет с роботом | `map`, вся карта |
| Слои | obstacle (`/scan`) + inflation | static (`/map` или `/projected_map`) + obstacle + inflation |
| Частота | 5 Гц | 1 Гц |
| Для чего | не врезаться прямо сейчас | планировать маршрут |

Footprint — прямоугольник `[[0.42, ±0.30], [-0.10, ±0.30]]`: колёса на оси
(x = 0), колея 0.60 м снаружи, каретка спереди до ≈0.42 м. **Должен совпадать
с `hoverbot.urdf.xacro`** — меняете размеры робота, правьте оба `footprint`
в `nav2_params.yaml`.

## 3. Octomap — честно о том, что он здесь делает

У робота **двумерный** лидар пылесоса. Octomap — трёхмерная структура, и из
плоского скана она получает один «слой» вокселей на высоте лидара (≈ 0.17 м
над `base_link`). Это работает и рабочий пайплайн готов, но **выигрыша
в 3D нет, пока нет 3D-датчика**.

Что даёт octomap уже сейчас:

- вероятностное накопление: шумный одиночный луч не превращается в стену;
- свободное пространство явно очищается лучами;
- `/projected_map` — готовая 2D-карта, которую можно отдать в global costmap
  (`use_octomap_costmap:=true`);
- готовая точка расширения под глубинную камеру или 3D-лидар.

Как подключить 3D-датчик: перенаправьте его `PointCloud2` в `octomap_server`
(`remappings=[('cloud_in', '/camera/points')]`), а для costmap добавьте
`VoxelLayer` с этим же облаком. `scan_to_cloud` тогда не нужен.

> **Грабли QoS.** `/projected_map` публикуется с `transient_local`
> (`latch: True`). Если выставить `latch: False`, статический слой costmap не
> получит карту (`incompatible QoS … DURABILITY`), и global costmap останется
> размером по умолчанию 5×5 м с предупреждением «Sensor origin … out of map
> bounds».

## 4. Сборка

```bash
sudo apt install ros-kilted-nav2-bringup ros-kilted-nav2-regulated-pure-pursuit-controller \
                 ros-kilted-octomap-server ros-kilted-octomap-rviz-plugins \
                 ros-kilted-laser-geometry ros-kilted-slam-toolbox

cd ~/ros2_ws_
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select hoverbot_nav
source install/setup.bash
```

`--symlink-install` важен: правки `nav2_params.yaml` и launch-файлов
подхватываются без пересборки (нужен лишь перезапуск launch).

Предупреждение CMake про `ament_target_dependencies` при сборке — безвредное.

## 5. Сценарии запуска

### 5.1. Симуляция: всё одной командой

```bash
ros2 launch hoverbot_nav sim_navigation.launch.py            # с окном Gazebo и RViz
ros2 launch hoverbot_nav sim_navigation.launch.py gui:=false rviz:=false   # без окон
```

Nav2 стартует через 10 с — когда Gazebo уже публикует `odom→base_link` и
slam_toolbox выдал `/map`. Подождите, пока в логе не появится
`Managed nodes are active`.

### 5.2. Симуляция: по частям (удобно отлаживать)

```bash
ros2 launch hoverbot_sim mapping.launch.py                   # терминал 1: Gazebo + SLAM
ros2 launch hoverbot_nav navigation.launch.py                # терминал 2: Nav2 + octomap
ros2 launch hoverbot_nav navigation.launch.py use_octomap_costmap:=true
```

### 5.3. Отправить цель

В RViz: кнопка **Nav2 Goal** (топик `/goal_pose`). Или из консоли:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Робот *не умеет ехать задним ходом* (`allow_reversing: false`): если цель позади,
он сначала развернётся на месте.

### 5.4. Реальное железо

Полный порядок — в [CHECKLIST.md](CHECKLIST.md#железо). Сам запуск:

```bash
# Wi-Fi: ПК и ESP32 в одной сети
ros2 launch hoverbot_nav robot_navigation.launch.py transport:=udp host:=192.168.0.105
# USB: ПК на роботе, ESP32 в порт «USB»
ros2 launch hoverbot_nav robot_navigation.launch.py transport:=serial device:=/dev/ttyACM0
# Бортовой ПК без экрана
ros2 launch hoverbot_nav robot_navigation.launch.py transport:=serial rviz:=false
```

Что запускается: `esp32_bridge` (лидар → `/scan`, одометрия → `/odom` и TF,
`/cmd_vel` → колёса) → через 8 с Nav2 и octomap с `use_sim_time:=false`.
slam_toolbox запускается внутри `esp32_bridge/robot.launch.py`.

**Шаги безопасности.** Первый выезд — на подставке. Прошивка ESP32 останавливает
колёса через 300 мс без команд, а плата гироскутера — по `SERIAL_TIMEOUT`
(≈0.8 с), так что при обрыве связи робот встанет, но не мгновенно.
Аварийный выключатель — под рукой.

Ходовые скорости в `nav2_params.yaml` консервативные (0.4 м/с рабочая,
0.5 м/с потолок). Поднимайте, только убедившись, что одометрия откалибрована
(процедура — в README).

## 6. Прошивки и что они значат для навигации

| Что | Где | Что важно знать |
|---|---|---|
| Прошивка ESP32-S3 | `firmware/esp32_bridge/` | Сборка `pio run -e wifi` или `-e usb`. Скорость, геометрия, пины — `include/config.h` |
| Прошивка гироскутера (STM32F103) | `firmware/hoverboard-firmware-hack-FOC/` (submodule) | Собирается под `VARIANT_USART`, прошивается через ST-Link |

**Согласованность значений** (иначе навигация врёт, а не просто не работает):

| Значение | ESP32 | ПК | Гироскутер |
|---|---|---|---|
| Колея 0.54 м | `WHEEL_BASE` | `wheel_base`, xacro | — |
| Диаметр 0.16 м | `WHEEL_DIAMETER` | `wheel_radius`×2 | — |
| Скорость UART | `HB_BAUD` = 1 000 000 | — | `USART2_BAUD`/`USART3_BAUD` |
| Footprint Nav2 | — | `nav2_params.yaml` | — |

> ⚠ **Найдено при составлении документации.** В submodule гироскутера
> `USART2_BAUD`/`USART3_BAUD` по умолчанию **115200**, а в
> `esp32_bridge/include/config.h` `HB_BAUD = 1000000`, и в submodule нет
> локальных правок. Если плата не прошита с изменённой скоростью, связь
> ESP32↔гироскутер не заработает. Решение — одно из двух (но не оба):
> добавить в `build_flags` окружения `VARIANT_USART` в
> `hoverboard-firmware-hack-FOC/platformio.ini` флаг `-DUSART2_BAUD=1000000`
> (или `USART3_BAUD` — смотря в какой кабель сайдборда включён ESP32), либо
> поставить `HB_BAUD = 115200` в `config.h` ESP32. Первое лучше: на
> 115200 телеметрия и команды на 20–50 Гц упираются в канал.

## 7. Диагностика

| Симптом | Причина | Что делать |
|---|---|---|
| `ros2 topic echo`, `ros2 node list` или launch зависают | Остатки Fast DDS shared memory после `kill -9` | `fastdds shm clean` или `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| `Node not found` у `ros2 lifecycle get` | Нода ещё не стартовала (после старта ждёт TF) | Подождать 20–30 с |
| `Invalid frame ID "odom"` в `local_costmap` | Нет TF `odom→base_link`: симуляция/мост ещё не поднялись | Запустить Nav2 позже или проверить мост |
| `Transform data too old … map to odom` в первые секунды | SLAM только стартует | Игнорировать; если не уходит — не публикуется `map→odom` |
| `Sensor origin … out of map bounds`, costmap 5×5 м | Не пришёл `/projected_map` (QoS) или `/map` | Проверить `latch: True`; `ros2 topic info /projected_map -v` |
| Робот крутится на месте и не едет | Цель позади, `allow_reversing: false` | Нормально, он развернётся; иначе проверьте `inflation_radius` — цель могла попасть в «запрещённую» зону |
| «Failed to make progress» | Footprint велик для прохода | Уменьшить `inflation_radius` или `cost_scaling_factor` |
| Nav2 не слушает `/cmd_vel` на роботе | Контроллер публикует `TwistStamped` | Убедиться, что `enable_stamped_cmd_vel: false` в трёх местах |
| На реальном роботе нет карты | `use_sim_time` у slam_toolbox = true | Запускать через `robot_navigation.launch.py` / `robot.launch.py` |

Полезные проверки:

```bash
ros2 lifecycle get /bt_navigator                       # active [3]
ros2 topic hz /scan /odom /cmd_vel                     # частоты
ros2 run tf2_tools view_frames                         # цепочка map→odom→base_link→laser
ros2 topic echo --once /global_costmap/costmap --field info
ros2 param get /global_costmap/global_costmap static_layer.map_topic
```

## 8. Как расширять

- **3D-датчик:** добавить мост топика в `bridge.yaml`, `VoxelLayer` в costmap,
  `cloud_in` octomap'а перенаправить на его облако.
- **Зоны запрета / ограничение скорости:** `KeepoutFilter`, `SpeedFilter`
  (нужен `nav2_map_server` для масок).
- **Локализация по готовой карте вместо SLAM:** `map_server` + `amcl`,
  при этом slam_toolbox выключить — иначе два `map→odom`.
- **Столкновения «в последний момент»:** `nav2_collision_monitor` между
  `velocity_smoother` и приводом.
