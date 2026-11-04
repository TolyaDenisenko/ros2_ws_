# Чек-лист: навигация hoverbot

Статус на 2026-11-04. ✅ сделано и проверено · 🟡 сделано, не проверено · ⬜ не сделано.

## Сделано

### Пакет и сборка
- ✅ Пакет `hoverbot_nav` (ament_cmake), собирается `colcon build`
- ✅ Нода `scan_to_cloud` (LaserScan → PointCloud2) на `laser_geometry`
- ✅ Зависимости объявлены в `package.xml`

### Nav2
- ✅ Параметры всех нод: controller (Regulated Pure Pursuit), planner (NavFn, A\*), smoother, behaviors, bt_navigator, velocity_smoother
- ✅ Все lifecycle-ноды переходят в `active` через `lifecycle_manager`
- ✅ Local costmap (obstacle + inflation) и global costmap (static + obstacle + inflation)
- ✅ `Twist`, а не `TwistStamped` — совместимость с `esp32_bridge` и Gazebo
- ✅ **Цель `/navigate_to_pose` в симуляции завершилась `SUCCEEDED`**
- ✅ `map_server`/`amcl` не запускаются — карту и `map→odom` даёт slam_toolbox

### Octomap
- ✅ `octomap_server` публикует `/octomap_binary`, `/octomap_full`, `/projected_map` (разрешение 5 см)
- ✅ `use_octomap_costmap:=true` переключает статический слой на `/projected_map` (проверено параметром ноды)
- ✅ Найдена и исправлена несовместимость QoS (`latch: True`)

### Сценарии запуска
- ✅ `navigation.launch.py` — Nav2 + octomap поверх SLAM (проверен в симуляции)
- ✅ `sim_navigation.launch.py` — всё в симуляции одной командой (аргументы разворачиваются; полный прогон не делался)
- 🟡 `robot_navigation.launch.py` — реальное железо (аргументы разворачиваются, на железе не запускался)

### Документация
- ✅ [NAV2_OCTOMAP.md](NAV2_OCTOMAP.md) — архитектура, ноды, сборка, сценарии, диагностика
- ✅ Этот чек-лист

## Нужно добавить

### Проверить в симуляции
- ⬜ Прогон `sim_navigation.launch.py` целиком (с таймером 10 с)
- ⬜ Объезд препятствия: цель за островом в `room.sdf`, смотреть на local costmap
- ⬜ Сравнить `/map` и `/projected_map` как источник global costmap (`use_octomap_costmap:=true`) — сглаживание при loop closure
- ⬜ Поведение при потере цели: срабатывание `spin`/`backup` из `behavior_server`

### Железо (⚠ ничего не проверялось — железа не было под рукой)
- ⬜ **Скорость UART:** `HB_BAUD = 1000000` в ESP32 против 115200 по умолчанию в прошивке гироскутера — привести к одному значению (см. раздел 6 в NAV2_OCTOMAP.md)
- ⬜ Прошить гироскутер под `VARIANT_USART` через ST-Link, убедиться в нужном кабеле сайдборда (USART2/USART3) и логике 3.3 В; **не подключать пин 15 В к ESP32**
- ⬜ **Пины лидара** 32/33 на ESP32-S3 — флеш/PSRAM, перенести на свободные GPIO (см. `PINOUT.md`)
- ⬜ Проверить, что `/scan`, `/odom`, TF приходят с реальной частотой (`ros2 topic hz`)
- ⬜ Калибровка одометрии (README): колея 0.54 м, диаметр 0.16 м
- ⬜ Первый выезд на подставке: убедиться, что `/cmd_vel` крутит колёса в нужную сторону и при обрыве связи колёса останавливаются
- ⬜ Подобрать скорости (`desired_linear_vel`, `max_velocity`) под реальную динамику
- ⬜ Подобрать `inflation_radius` под реальные проёмы (двери ≈ 0.8–0.9 м при ширине робота 0.60)
- ⬜ Сверить footprint с реальными габаритами (колёсико кастера и площадка)

### Функциональность
- ⬜ RViz-конфиг с панелью Nav2, costmap'ами, путём и `OcTree`
- ⬜ 3D-датчик (глубинная камера / 3D-лидар) — без него octomap остаётся «одним слоем»
- ⬜ `nav2_collision_monitor` перед приводом как страховка
- ⬜ Режим «навигация по готовой карте»: `map_server` + `amcl` вместо slam_toolbox
- ⬜ Автотесты: smoke-тест, что lifecycle-ноды становятся `active` (сейчас проверка ручная)
- ⬜ Обновить README.md ссылкой на `docs/`

## Известные ограничения
- Octomap строится из 2D-скана → один слой вокселей на высоте лидара.
- Робот не ездит задним ходом (`allow_reversing: false`).
- Footprint — прямоугольник; торчащие элементы не учитываются.
- В первые секунды после старта в логе `controller_server` бывает ошибка `Transform data too old … map to odom` — это ожидание SLAM, не сбой.
