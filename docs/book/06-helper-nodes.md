# Глава 6. Вспомогательные ноды

> **Цель главы:** разобрать две маленькие ноды проекта и на их примере —
> минимальный шаблон ноды на rclcpp.

## 6.1. Шаблон ноды на rclcpp

Обе ноды устроены одинаково, и этот шаблон стоит запомнить:

```cpp
class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    const auto in = declare_parameter("input_topic", "/in");   // 1. параметры
    pub_ = create_publisher<Out>(out_topic, 10);                // 2. издатели
    sub_ = create_subscription<In>(in, 10,                      // 3. подписки
      [this](In::ConstSharedPtr msg) { ...; pub_->publish(out); });
  }
private:
  rclcpp::Publisher<Out>::SharedPtr pub_;
  rclcpp::Subscription<In>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
```

- `declare_parameter(имя, значение_по_умолчанию)` возвращает значение с
  учётом `--ros-args -p` и YAML; тип выводится из значения по умолчанию.
- Число `10` — глубина очереди QoS (history depth).
- Лямбда захватывает `this`; подписка живёт, пока жив `sub_`.

## 6.2. `scan_to_cloud` (пакет `hoverbot_nav`)

**Зачем.** `octomap_server` строит 3D-карту из облаков точек
(`PointCloud2`), а лидар даёт `LaserScan`. Нода переводит одно в другое.

**Интерфейс.**

| | Имя (по умолч.) | Тип | QoS |
|---|---|---|---|
| подписка | `scan` (`scan_topic`) | `LaserScan` | `SensorDataQoS` (best effort) |
| публикация | `scan_cloud` (`cloud_topic`) | `PointCloud2` | reliable, depth 5 |

**Код.**

```cpp
sub_ = create_subscription<LaserScan>(scan_topic, rclcpp::SensorDataQoS(),
  [this](LaserScan::ConstSharedPtr scan) {
    PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);   // laser_geometry
    pub_->publish(cloud);
  });
```

Три решения, которые стоит понять:

1. **Облако остаётся в кадре лидара.** `projectLaser` не трогает TF — он
   просто переводит полярные координаты `(угол, дальность)` в декартовы
   `(x, y, 0)` в кадре `laser`. Переводом в `map` занимается сам
   `octomap_server` — ему нужна позиция сенсора как начало лучей для
   трассировки свободного пространства.
2. **Лучи `inf`/`NaN` отбрасываются** внутри `projectLaser`. Значит, octomap
   не узнает о свободном пространстве вдоль «пустых» лучей — это ограничение
   2D-лидара в связке с octomap.
3. **QoS.** Подписка `SensorDataQoS` (best effort) совместима с любым
   издателем `/scan`. Публикация reliable — совместима с подписчиками и
   reliable, и best effort. Несовпадение QoS — частая причина «топик есть, а
   данных нет»; проверяйте `ros2 topic info -v`.

**Запуск** — из `navigation.launch.py` (аргумент `octomap:=true`), отдельно:

```bash
ros2 run hoverbot_nav scan_to_cloud --ros-args -p scan_topic:=/scan
```

## 6.3. `twist_to_twist_stamped` (пакет `twist_converter`)

**Зачем.** В Jazzy и новее `diff_drive_controller` из ros2_control принимает
только `TwistStamped`, а teleop и многие старые ноды шлют `Twist`. Нода
добавляет заголовок.

**Параметры.**

| Параметр | По умолчанию |
|---|---|
| `input_topic` | `/cmd_vel_unstamped` |
| `output_topic` | `/diff_drive_controller/cmd_vel` |
| `frame_id` | `base_link` |

**Код** — один колбэк:

```cpp
TwistStamped out;
out.header.stamp = now();
out.header.frame_id = frame_id_;
out.twist = *msg;
pub_->publish(out);
```

**Когда нужна.** В текущих launch-файлах **не используется**: мост и Gazebo
DiffDrive принимают обычный `Twist`, а Nav2 настроен на `Twist`
(`enable_stamped_cmd_vel: false`, гл. 9). Пригодится, если перейти на
ros2_control:

```bash
ros2 run twist_converter twist_to_twist_stamped --ros-args \
  -p input_topic:=/cmd_vel -p output_topic:=/diff_drive_controller/cmd_vel
```

## 6.4. Как добавить свою ноду в пакет

На примере `hoverbot_nav`:

1. Файл `src/hoverbot_nav/src/my_node.cpp` по шаблону 6.1.
2. В `CMakeLists.txt`:
   ```cmake
   add_executable(my_node src/my_node.cpp)
   ament_target_dependencies(my_node rclcpp sensor_msgs)   # или target_link_libraries
   install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
   ```
3. Зависимости — в `package.xml` (`<depend>sensor_msgs</depend>`).
4. `colcon build --packages-select hoverbot_nav && source install/setup.bash`.
5. `ros2 run hoverbot_nav my_node`.

> **✎ Упражнение.** Напишите ноду `scan_filter`, которая заменяет на `inf`
> все точки `/scan` ближе 0.3 м (отражения от корпуса робота) и публикует
> `/scan_filtered`. Подключите её к SLAM через параметр `scan_topic` в
> `slam_param.yaml`. Какой QoS выбрать для подписки и публикации?
