# Приложение A. Величины, которые должны совпадать

Одна физическая величина часто записана в нескольких местах разных кодовых
баз. Расхождение не вызывает ошибок компиляции — только тихо портит
одометрию, карту или связь. `scripts/hardware_bringup.sh check` проверяет
отмеченные ✓.

| Величина | Значение | Где записана | Проверка |
|---|---|---|---|
| Колея | 0.54 м | `config.h: WHEEL_BASE`; параметр моста `wheel_base`; xacro `wheel_separation` | ✓ |
| Диаметр колеса | 0.16 м | `config.h: WHEEL_DIAMETER`; xacro `wheel_radius` × 2 | ✓ |
| Скорость UART платы | 1 000 000 или 115 200 | `config.h: HB_BAUD`; FOC `USART2_BAUD` (флаг или `Inc/config.h`) | ✓ |
| Положение лидара | x 0.10, z 0.17 от оси | параметр `lidar_xyz`; xacro `lidar_x`, `lidar_height − wheel_radius` | — |
| Порт UDP | 8888 | `config.h: UDP_PORT`; параметр `port` | — |
| Формат протокола | v2 | `protocol.h`; `protocol.hpp`/`protocol.cpp` | gtest |
| Длина кадра телеметрии FOC | 18 байт | `hbReadFeedback`; FOC `ENABLE_ODOMETRY` выключен | — |
| Параметры SLAM | | `slam_sim.yaml` ↔ `slam_param.yaml` (кроме `use_sim_time`) | ✓ |
| Footprint | 0.42 / −0.10 / ±0.30 | `nav2_params.yaml`, **оба** costmap'а; габариты xacro | — |
| Дальность лидара | 6 м | SLAM `max_laser_range`; costmap `obstacle_max_range`; octomap `sensor_model.max_range` | — |
| Кадры | `odom`, `base_link`, `laser`, `map` | мост; DiffDrive и `gz_frame_id` в xacro; SLAM; Nav2 | — |
| Тип `/cmd_vel` | `Twist` | мост; DiffDrive; `enable_stamped_cmd_vel: false` в Nav2 | — |
