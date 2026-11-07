# Глава 9. Навигация: плагины Nav2 и octomap

> **Цель главы:** понять, из каких серверов и плагинов собран Nav2 в пакете
> `hoverbot_nav`, как настроить каждый плагин под тележку, и как работает
> octomap.

Подробный обзор архитектуры есть в [../NAV2_OCTOMAP.md](../NAV2_OCTOMAP.md);
здесь — учебный разбор с упором на настройку.

## 9.1. Nav2 как набор плагинов

Nav2 — это несколько **серверов** (lifecycle-нод), каждый из которых
загружает **плагины** через pluginlib. Сервер задаёт роль, плагин —
алгоритм. В YAML это всегда выглядит одинаково:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]          # список ИМЁН экземпляров
    FollowPath:                                 # имя экземпляра
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4                   # параметры плагина
```

Имя экземпляра (`FollowPath`) — произвольное, но на него ссылается дерево
поведения bt_navigator. Поэтому стандартные имена (`FollowPath`, `GridBased`)
лучше не менять.

| Сервер | Плагин у нас | Альтернативы |
|---|---|---|
| `planner_server` | `nav2_navfn_planner::NavfnPlanner` | `SmacPlanner2D`, `SmacPlannerHybrid`, `ThetaStarPlanner` |
| `controller_server` | `RegulatedPurePursuitController` | `DWBLocalPlanner`, `MPPIController` |
| `smoother_server` | `nav2_smoother::SimpleSmoother` | `SavitzkyGolaySmoother`, `ConstrainedSmoother` |
| `behavior_server` | `Spin`, `BackUp`, `DriveOnHeading`, `Wait` | `AssistedTeleop` |
| `bt_navigator` | `NavigateToPoseNavigator`, `NavigateThroughPosesNavigator` | |
| costmap-слои | `StaticLayer`, `ObstacleLayer`, `InflationLayer` | `VoxelLayer`, `RangeSensorLayer` |
| progress/goal checker | `SimpleProgressChecker`, `SimpleGoalChecker` | `PoseProgressChecker`, `StoppedGoalChecker` |

## 9.2. Запуск

```bash
ros2 launch hoverbot_nav sim_navigation.launch.py               # симуляция целиком
ros2 launch hoverbot_nav robot_navigation.launch.py transport:=udp host:=<IP>   # ⚠ робот поедет
# или поверх уже запущенного SLAM:
ros2 launch hoverbot_nav navigation.launch.py use_sim_time:=false
```

Цель — кнопка **Nav2 Goal** в RViz или:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### Что делает `navigation.launch.py`

1. **`RewrittenYaml`** подставляет `use_sim_time` и выбирает топик
   статического слоя (`/map` или `/projected_map` по аргументу
   `use_octomap_costmap`). `use_sim_time` дополнительно передаётся отдельным
   словарём: в YAML такого ключа нет, а `RewrittenYaml` добавляет только
   существующие.
2. Запускает шесть lifecycle-нод с переназначениями:
   ```
   controller_server ──cmd_vel_nav──► velocity_smoother ──cmd_vel──► мост / Gazebo
   behavior_server   ──cmd_vel_nav──┘
   ```
3. **`lifecycle_manager`** стартует через 4 с (`TimerAction`): если он
   позовёт `configure` раньше, чем обнаружены сервисы нод, bringup
   обрывается.
4. `scan_to_cloud` + `octomap_server` (если `octomap:=true`).

Обёртки ждут, пока поднимется остальное: `sim_navigation` — 10 с (Gazebo
грузится долго), `robot_navigation` — 8 с.

### Чего нет и почему

- **`map_server` и `amcl`** — карту и `map → odom` даёт slam_toolbox. Второй
  источник `map → odom` ломает граф SLAM.
- **`TwistStamped`** — в Kilted Nav2 по умолчанию публикует `TwistStamped`,
  а мост и DiffDrive ждут `Twist`. Поэтому в `controller_server`,
  `behavior_server` и `velocity_smoother` стоит
  `enable_stamped_cmd_vel: false`.

## 9.3. Costmap'ы

```
        local_costmap                    global_costmap
  кадр  odom, окно 4×4 м с роботом       map, вся карта
  слои  obstacle + inflation             static + obstacle + inflation
  кто   controller_server                planner_server
```

### Footprint — должен совпадать с моделью

```yaml
footprint: "[[0.42, 0.30], [0.42, -0.30], [-0.10, -0.30], [-0.10, 0.30]]"
```

Прямоугольник в кадре `base_link`: вперёд до 0.42 м (площадка + кастер),
назад до −0.10 м, по ширине ±0.30 м (колёса снаружи 0.60 м). Задан в
**обоих** costmap'ах. При изменении геометрии (гл. 7.2) поправьте оба.

### StaticLayer

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_topic: /map
  map_subscribe_transient_local: true
```

Берёт карту от slam_toolbox. `transient_local` — чтобы получить последнюю
опубликованную карту даже при подписке позже публикации.

### ObstacleLayer

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan:
    topic: /scan
    data_type: "LaserScan"
    marking: true            # отмечать препятствия
    clearing: true           # стирать лучами свободное пространство
    obstacle_max_range: 6.0  # дальше — не отмечать
    raytrace_max_range: 8.0  # стирать до 8 м
```

Правило: `raytrace_max_range` ≥ `obstacle_max_range`, иначе от удалившегося
препятствия останется «призрак». Лучи `inf` (нет отражения) в Nav2 по
умолчанию не стирают — для лидара пылесоса это нормально.

### InflationLayer

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 0.65
  cost_scaling_factor: 3.0
```

Вокруг препятствий — «стоимость» близости, спадающая экспоненциально.
`inflation_radius` должен быть больше радиуса описанной окружности робота
(≈0.5 м для нашего footprint), иначе планировщик будет прижимать путь к
стенам. Но в двери шириной 0.8 м робот шириной 0.6 м проходит с зазором
по 0.1 м — слишком большой радиус «закроет» проём. Если робот отказывается
ехать в дверь: уменьшите `inflation_radius` до 0.5 или увеличьте
`cost_scaling_factor` (стоимость спадает быстрее).

## 9.4. Планировщик: NavFn

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: true          # A* вместо Дейкстры — быстрее
  allow_unknown: true      # можно планировать через неисследованное
  tolerance: 0.5           # если цель занята — ближайшая свободная в 0.5 м
```

`allow_unknown: true` нужен при исследовании: SLAM ещё не видел пространство
за углом. NavFn считает робота точкой (футпринт учитывает inflation).
Для длинного робота в узких местах лучше `SmacPlannerHybrid` с учётом формы
— это тема для развития.

## 9.5. Контроллер: Regulated Pure Pursuit

Pure Pursuit — «гонка за морковкой»: на пути выбирается точка на расстоянии
`lookahead` и вычисляется дуга, ведущая к ней. Regulated-версия ещё и
сбрасывает скорость на поворотах и около препятствий.

| Параметр | Значение | Как настраивать |
|---|---|---|
| `desired_linear_vel` | 0.4 | крейсерская скорость, м/с |
| `lookahead_dist` | 0.6 | больше — плавнее, но срезает углы |
| `min/max_lookahead_dist` | 0.3 / 0.9 | границы при `use_velocity_scaled_lookahead_dist` |
| `lookahead_time` | 1.5 | lookahead = скорость × время |
| `use_rotate_to_heading` | true | сначала развернуться на месте, потом ехать |
| `rotate_to_heading_min_angle` | 0.785 | разворот, если угол к пути > 45° |
| `rotate_to_heading_angular_vel` | 0.8 | скорость разворота, рад/с |
| `max_angular_accel` | 1.5 | |
| `use_regulated_linear_velocity_scaling` | true | замедление на крутых поворотах |
| `regulated_linear_scaling_min_radius` | 0.9 | радиус, с которого начинается замедление |
| `use_collision_detection` | true | проверка столкновений вдоль пути |
| `allow_reversing` | false | задом не ездит |

Дифференциальный робот разворачивается на месте — `use_rotate_to_heading`
это использует. Для тяжёлого гироскутера на ковре минимальная угловая
скорость, при которой колёса вообще трогаются, может быть выше 0.3 рад/с —
подберите на железе.

`controller_server` общие:

```yaml
controller_frequency: 20.0
min_y_velocity_threshold: 0.5        # у дифф. привода нет боковой скорости
progress_checker:  required_movement_radius: 0.5, movement_time_allowance: 10.0
general_goal_checker: xy_goal_tolerance: 0.15, yaw_goal_tolerance: 0.25
```

Если робот 10 с не сместился на 0.5 м — progress checker объявляет провал,
и дерево поведения запускает восстановление (spin, backup).

## 9.6. Сглаживатель скорости

```yaml
velocity_smoother:
  max_velocity: [0.5, 0.0, 1.0]       # [vx, vy, ωz]
  min_velocity: [-0.3, 0.0, -1.0]
  max_accel:    [0.6, 0.0, 1.5]
  max_decel:    [-0.8, 0.0, -2.0]
  feedback: "OPEN_LOOP"
  smoothing_frequency: 20.0
```

Последний рубеж перед колёсами: ограничивает ускорения, чтобы тяжёлые
мотор-колёса не буксовали и одометрия не врала. `OPEN_LOOP` — считать от
прошлой команды, а не от `/odom` (одометрия на 50 Гц шумит).
`smoothing_frequency` 20 Гц совпадает с частотой отправки команд ESP32.

## 9.7. Behavior server и bt_navigator

Восстановительные поведения: `spin` (поворот на месте), `backup` (откат),
`drive_on_heading`, `wait`. Ограничения: `max_rotational_vel: 1.0`,
`rotational_acc_lim: 1.5`.

> `allow_reversing: false` у контроллера не мешает `backup` ехать назад. Если
> сзади робота нет датчика (лидар закрыт коробом?) — уберите `backup` из
> `behavior_plugins`.

`bt_navigator` использует дерево поведения по умолчанию из `nav2_bt_navigator`:
планировать → следовать → при сбое очистить costmap'ы и попробовать
восстановление.

## 9.8. Octomap

```
/scan → scan_to_cloud → /scan_cloud → octomap_server → /octomap_binary  (3D)
                                                      → /projected_map  (2D)
```

Параметры `octomap_server` (в launch-файле, а не в YAML):

| Параметр | Значение | Смысл |
|---|---|---|
| `frame_id` | `map` | кадр карты |
| `base_frame_id` | `base_link` | |
| `resolution` | 0.05 | размер вокселя |
| `latch` | **True** | transient_local — без этого static_layer не получает `/projected_map` |
| `sensor_model.max_range` | 6.0 | = `max_laser_range` SLAM |
| `filter_ground_plane` | False | |
| `occupancy_min_z` / `max_z` | −0.1 / 0.6 | какой слой вокселей проецировать в 2D |

`use_octomap_costmap:=true` переключает статический слой global costmap на
`/projected_map`.

> **Честно об ограничении.** Octomap строится из **2D**-скана, поэтому
> 3D-карта — один слой вокселей на высоте лидара. Пользы над `/map` пока
> мало. Инфраструктура готова для 3D-датчика (глубинная камера): его облако
> подаётся в `octomap_server` вместо `/scan_cloud`.

## 9.9. Старый файл `nav_param.yaml`

`src/esp32_bridge/config/nav_param.yaml` — более ранняя версия параметров
Nav2. **Ни один launch-файл его не использует**; актуальный файл —
`src/hoverbot_nav/config/nav2_params.yaml`. Не правьте старый, ожидая
эффекта.

## 9.10. Отладка

```bash
ros2 lifecycle get /controller_server    # active [3] — для всех шести
ros2 topic echo /cmd_vel                 # выходит ли команда
ros2 topic hz /local_costmap/costmap
```

| Симптом | Причина |
|---|---|
| `Transform data too old … map to odom` первые секунды | SLAM ещё не опубликовал — норма |
| Ноды не становятся `active` | lifecycle_manager стартовал раньше — увеличить `TimerAction` |
| План есть, робот не едет | `/cmd_vel` в `TwistStamped` — проверить `enable_stamped_cmd_vel` |
| Робот крутится на месте у цели | `yaw_goal_tolerance` мала или угловая скорость ниже порога трогания |
| Global costmap пустой при `use_octomap_costmap` | `latch` у octomap_server выключен |

> **✎ Упражнение.** Замените контроллер на MPPI (`nav2_mppi_controller::MPPIController`,
> `motion_model: "DiffDrive"`). Не меняя имя экземпляра `FollowPath`, добейтесь
> проезда вокруг острова в `room.sdf`. Сравните плавность с RPP в RViz.
