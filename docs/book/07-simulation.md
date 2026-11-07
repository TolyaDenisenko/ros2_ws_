# Глава 7. Симуляция: модель, плагины Gazebo, мост топиков

> **Цель главы:** понять, как устроена модель робота `hoverbot.urdf.xacro`,
> что делает каждый плагин Gazebo и как их настроить, как связаны Gazebo и
> ROS 2.

Пакет `src/hoverbot_sim`:

```
urdf/hoverbot.urdf.xacro   модель робота (все размеры — в начале файла)
worlds/room.sdf            мир: комната 8×6 м с островом
config/bridge.yaml         какие топики переходят Gazebo ↔ ROS
config/slam_sim.yaml       slam_toolbox для симуляции (гл. 8)
launch/sim.launch.py       Gazebo + робот + мост + RViz
launch/mapping.launch.py   то же + slam_toolbox
rviz/sim.rviz, mapping.rviz
```

## 7.1. Запуск

```bash
ros2 launch hoverbot_sim sim.launch.py              # только симуляция
ros2 launch hoverbot_sim mapping.launch.py          # + SLAM
ros2 launch hoverbot_sim mapping.launch.py gui:=false rviz:=false   # без окон
ros2 run teleop_twist_keyboard teleop_twist_keyboard                # управление
```

Что делает `sim.launch.py` по шагам:

1. **xacro → URDF** при запуске: `Command(['xacro ', xacro_file])`. Поэтому
   правки размеров в `.xacro` подхватываются без пересборки (при
   `--symlink-install`).
2. **Gazebo** через `ros_gz_sim/gz_sim.launch.py` с аргументами `-r <мир>`
   (сразу запустить физику) и `-s` при `gui:=false` (только сервер).
3. **`robot_state_publisher`** публикует `/robot_description` и TF
   неподвижных звеньев (`base_link → laser`, `base_link → caster`) и колёс
   (по `/joint_states`).
4. **`ros_gz_sim create`** берёт модель из `/robot_description` и ставит её в
   точку (−2.5, 0, 0.1) — у западной стены.
5. **`ros_gz_bridge parameter_bridge`** с `config/bridge.yaml`.
6. **RViz** с `use_sim_time: true`.

Все ноды симуляции работают на **времени симуляции** (`use_sim_time: true`,
топик `/clock`). Если хотя бы одна нода на системном времени — TF «из
будущего/прошлого» и предупреждения `Lookup would require extrapolation`.

## 7.2. Модель робота: `hoverbot.urdf.xacro`

### Параметры

Все размеры собраны в блоке `xacro:property` в начале файла. Менять **только
там** — остальное считается из них.

| Параметр | Значение | Смысл | Связан с |
|---|---|---|---|
| `wheel_radius` | 0.08 | радиус колеса, м | `WHEEL_DIAMETER`/2 в ESP32 |
| `wheel_separation` | 0.54 | колея, м | `WHEEL_BASE`, `wheel_base` |
| `wheel_width` | 0.06 | ширина шины | |
| `deck_length`, `deck_width` | 0.50, 0.30 | площадка | footprint Nav2 |
| `deck_rear_x` | −0.08 | задний край площадки от оси | footprint Nav2 |
| `caster_x`, `caster_radius` | 0.37, 0.03 | кастер (TODO: замерить) | |
| `lidar_x`, `lidar_height` | 0.10, 0.25 | лидар: вперёд от оси, высота от пола | `lidar_xyz` моста |
| `lidar_rate` | 5 | об/с | |
| `lidar_samples` | 360 | точек на оборот | `scan_bins` моста |
| `lidar_range_min/max` | 0.15 / 8.0 | дальность, м | |
| `lidar_noise` | 0.01 | СКО шума дальности, м | |

Производные:

```xml
<xacro:property name="lidar_z"  value="${lidar_height - wheel_radius}"/>   <!-- от оси, а не от пола -->
<xacro:property name="caster_z" value="${caster_radius - wheel_radius}"/>
```

`base_link` — на высоте оси колёс, поэтому высоту лидара от пола переводят в
высоту от оси: 0.25 − 0.08 = 0.17 м. Это ровно `lidar_xyz[2]` у моста.

### Звенья и шарниры

```
base_link ── left_wheel_joint  (continuous, ось Y) ── left_wheel
          ── right_wheel_joint (continuous, ось Y) ── right_wheel
          ── caster_joint (fixed) ── caster          (шар без трения)
          ── laser_joint  (fixed) ── laser
```

- Колёса — цилиндры, повёрнутые `rpy="π/2 0 0"` (цилиндр в URDF стоит вдоль
  Z, колесо — вдоль Y).
- Инерция задаётся честно (макрос `box_inertia`, формулы цилиндра): с
  нулевой или неправдоподобной инерцией физика Gazebo «взрывается».
- **Кастер — шар с нулевым трением** (`mu1 = mu2 = 0`). Это стандартный приём:
  моделировать поворотное колесо шарниром сложно и нестабильно, а шар без
  трения скользит в любую сторону, как настоящий кастер.
- Трение колёс `mu = 1.0`.

## 7.3. Сенсоры

Сенсоры — это не плагины модели, а блоки `<sensor>`, которые обслуживают
**системы мира** (`gz-sim-sensors-system`, `gz-sim-imu-system` в `room.sdf`).
Без этих систем в мире сенсоры молчат.

### Лидар (`gpu_lidar`)

```xml
<gazebo reference="laser">
  <sensor name="lidar" type="gpu_lidar">
    <topic>scan</topic>
    <gz_frame_id>laser</gz_frame_id>
    <update_rate>${lidar_rate}</update_rate>
    <lidar>
      <scan><horizontal>
        <samples>${lidar_samples}</samples>
        <min_angle>${-pi}</min_angle>
        <max_angle>${pi - 2 * pi / lidar_samples}</max_angle>
      </horizontal></scan>
      <range><min>…</min><max>…</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><stddev>${lidar_noise}</stddev></noise>
    </lidar>
  </sensor>
</gazebo>
```

Как настраивать:

- **`gz_frame_id`** — критичен. Без него Gazebo подпишет скан кадром вида
  `hoverbot/laser/lidar`, которого нет в TF, и SLAM будет молча
  игнорировать сканы.
- **`max_angle = π − шаг`**, а не π: иначе первая и последняя точки
  совпадут по углу. Так же устроен `/scan` реального моста (−π … π − 1°).
- **`gpu_lidar`** считается на видеокарте (рендер-движок `ogre2`). На машине
  без GPU в режиме `gui:=false` он всё равно работает через программный
  рендеринг, но медленнее.
- Чтобы симуляция была ближе к лидару пылесоса, можно увеличить
  `lidar_noise` до 0.02–0.03 и уменьшить `lidar_range_max` до 6.

### IMU

```xml
<gazebo reference="base_link">
  <sensor name="imu" type="imu">
    <topic>imu</topic>
    <gz_frame_id>base_link</gz_frame_id>
    <update_rate>100</update_rate>
  </sensor>
</gazebo>
```

Публикуется в `/imu`, но сейчас его никто не использует: на реальном роботе
IMU работает внутри ESP32 (удержание курса) и в ROS не публикуется.

## 7.4. Плагины модели

### DiffDrive — привод и одометрия

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>${wheel_separation}</wheel_separation>
  <wheel_radius>${wheel_radius}</wheel_radius>
  <max_linear_acceleration>1.0</max_linear_acceleration>
  <max_angular_acceleration>2.0</max_angular_acceleration>
  <topic>cmd_vel</topic>
  <odom_topic>odom</odom_topic>
  <tf_topic>tf</tf_topic>
  <frame_id>odom</frame_id>
  <child_frame_id>base_link</child_frame_id>
  <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

Это аналог пары «ESP32 + мост» в симуляции: принимает `Twist`, крутит колёса,
публикует `/odom` и TF `odom → base_link` с теми же именами кадров.

| Тег | Как настроить |
|---|---|
| `wheel_separation`, `wheel_radius` | из параметров xacro — не трогать |
| `max_linear_acceleration`, `max_angular_acceleration` | ограничения ускорения. Тяжёлые мотор-колёса разгоняются не мгновенно; подберите по реальному роботу, чтобы контроллер Nav2 в симуляции не был «слишком хорош» |
| `max_linear_velocity`, `max_angular_velocity` (не заданы) | можно добавить, чтобы ограничить скорость как у реального робота |
| `odom_publish_frequency` | 50 Гц — как у ESP32 |
| `frame_id`, `child_frame_id` | `odom`, `base_link` — как у моста |

> **Отличие от железа:** DiffDrive выполняет последнюю команду **вечно**.
> ESP32 останавливается через 300 мс. Для теста аварийной остановки
> симуляция не подходит.

Одометрия DiffDrive **идеальна** (колёса не проскальзывают). Чтобы проверить
SLAM на реалистичной одометрии, можно уменьшить `mu` колёс или слегка
изменить `wheel_radius` в плагине относительно геометрии — это внесёт
систематическую ошибку, как у реального робота.

### JointStatePublisher

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <topic>joint_states</topic>
</plugin>
```

Публикует углы колёс, `robot_state_publisher` превращает их в TF колёс — без
этого колёса в RViz «отрываются» от корпуса.

## 7.5. Мир: `room.sdf`

Комната 8 × 6 м с «островом» посередине: вокруг него можно объехать по
кругу и проверить замыкание петли. Мебель делает геометрию несимметричной —
чтобы scan matching не путал одинаковые участки.

Системы мира, обязательные для модели:

| Плагин | Зачем |
|---|---|
| `gz-sim-physics-system` | физика |
| `gz-sim-user-commands-system` | создание модели (`create`) |
| `gz-sim-scene-broadcaster-system` | картинка в GUI |
| `gz-sim-sensors-system` (`ogre2`) | лидар |
| `gz-sim-imu-system` | IMU |

Шаг физики 1 мс, `real_time_factor` 1.0. Если машина не тянет (RTF в углу
окна Gazebo < 0.5), уменьшите `lidar_samples` или запустите `gui:=false`.

## 7.6. Мост Gazebo ↔ ROS: `bridge.yaml`

```yaml
- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
```

| Топик | Направление | Тип ROS ↔ Gazebo |
|---|---|---|
| `/clock` | GZ→ROS | `rosgraph_msgs/Clock` ↔ `gz.msgs.Clock` |
| `/scan` | GZ→ROS | `LaserScan` ↔ `gz.msgs.LaserScan` |
| `/odom` | GZ→ROS | `Odometry` ↔ `gz.msgs.Odometry` |
| `/tf` | GZ→ROS | `TFMessage` ↔ `gz.msgs.Pose_V` |
| `/joint_states` | GZ→ROS | `JointState` ↔ `gz.msgs.Model` |
| `/imu` | GZ→ROS | `Imu` ↔ `gz.msgs.IMU` |
| `/cmd_vel` | ROS→GZ | `Twist` ↔ `gz.msgs.Twist` |

Как добавить топик: допишите блок, перезапустите launch. Узнать имя и тип в
Gazebo: `gz topic -l`, `gz topic -i -t /scan`.

> **⚠ Ловушка `/tf`.** Через мост идёт **только** `odom → base_link` от
> DiffDrive. Не добавляйте в мир плагин `PosePublisher` с выводом в `/tf` —
> появится второй источник поз, и TF-дерево поломается.

## 7.7. Как изменить робота

Пример: лидар подняли на 5 см.

1. Xacro: `lidar_height` 0.25 → 0.30.
2. Мост: `lidar_xyz` → `[0.10, 0.0, 0.22]` (= 0.30 − 0.08).
3. Перезапустить launch — пересборка не нужна.
4. Проверить: `ros2 run tf2_ros tf2_echo base_link laser`.

> **✎ Упражнение.** Добавьте в `room.sdf` коробку 0.3×0.3×0.15 м — ниже
> плоскости лидара (0.25 м). Увидит ли её SLAM? А Nav2? Что это говорит о
> необходимости 3D-датчика (см. гл. 9, octomap)?
