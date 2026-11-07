# Глава 8. SLAM: slam_toolbox и его параметры

> **Цель главы:** понять, как slam_toolbox строит карту из `/scan` и
> `/odom`, как его запускать (lifecycle!) и что означает каждый параметр в
> `slam_param.yaml`.

## 8.1. Как работает slam_toolbox (в двух абзацах)

slam_toolbox — графовый SLAM. Робот едет, одометрия даёт грубую оценку
движения. Когда робот проехал достаточно (`minimum_travel_distance` или
`minimum_travel_heading`), берётся скан и **сопоставляется** (scan matching)
с предыдущими сканами в окрестности позы из одометрии. Уточнённая поза
становится **узлом графа**, связь с предыдущим узлом — **ребром**.

Когда робот возвращается в уже виденное место, slam_toolbox находит
совпадение скана с давними узлами — **замыкание петли** (loop closure) — и
добавляет ребро между ними. Решатель (Ceres) перераспределяет накопленную
ошибку по всему графу, карта «защёлкивается». Итог публикуется как `/map`
(`OccupancyGrid`) и поправка TF `map → odom`.

```
вход:  /scan,  TF odom→base_link→laser
выход: /map,   TF map→odom,   /slam_toolbox/graph_visualization
```

## 8.2. Запуск: lifecycle-нода

Начиная с Jazzy slam_toolbox — **lifecycle-нода**. Запущенная как обычный
`Node(...)`, она остаётся в состоянии `unconfigured` и ничего не делает.
Поэтому оба launch-файла проекта подключают штатный
`slam_toolbox/launch/online_async_launch.py` с `autostart: true`:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
    launch_arguments={
        'slam_params_file': ...,
        'use_sim_time': 'false',      # ⚠ по умолчанию там true!
        'autostart': 'true',
    }.items())
```

Проверка:

```bash
ros2 lifecycle get /slam_toolbox      # active [3]
```

Режимы: `online_async` (используется у нас) обрабатывает последний скан,
пропуская устаревшие, — подходит для слабого ПК. `online_sync` обрабатывает
каждый скан — точнее, но может отставать.

## 8.3. Два файла параметров

| Файл | Где используется | Отличие |
|---|---|---|
| `src/hoverbot_sim/config/slam_sim.yaml` | `mapping.launch.py` | `use_sim_time: true` |
| `src/esp32_bridge/config/slam_param.yaml` | `robot.launch.py` | `use_sim_time: false` |

Всё остальное **обязано совпадать** — иначе то, что отлажено в симуляции, не
перенесётся на робота. `scripts/hardware_bringup.sh check` это проверяет.

Свой файл для эксперимента в симуляции:

```bash
ros2 launch hoverbot_sim mapping.launch.py slam_params:=/путь/к/my_slam.yaml
```

## 8.4. Параметры: что и как настраивать

Комментарии «было:» в YAML — значения прежней версии, которые мешали
замыканию петель. Они оставлены как урок.

### Плагин решателя

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
ceres_preconditioner: SCHUR_JACOBI
ceres_trust_strategy: LEVENBERG_MARQUARDT
ceres_dogleg_type: TRADITIONAL_DOGLEG
ceres_loss_function: None
```

Решатель графа загружается как **pluginlib-плагин**. Ceres — стандартный и
единственный, который стоит использовать. `SPARSE_NORMAL_CHOLESKY` подходит
для разреженного графа поз. `ceres_loss_function: HuberLoss` делает решатель
устойчивее к ложным замыканиям ценой более медленной сходимости — включайте,
если карта «ломается» после неверного замыкания.

### Кадры и топики

```yaml
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
mode: mapping          # или localization
```

Одинаковы для робота и симуляции — в этом смысл двойника.

### Общие

| Параметр | Значение | Как настраивать |
|---|---|---|
| `resolution` | 0.05 | размер клетки карты, м. 0.025 — детальнее, но больше ПК и шума |
| `max_laser_range` | 6.0 | лучи дальше обрезаются. Было 2.5 — мало геометрии для петель. Не больше реальной надёжной дальности лидара |
| `min_laser_range` | 0.15 | отсекает отражения от корпуса |
| `minimum_time_interval` | 0.2 | не чаще, с. Было 0.01 |
| `transform_publish_period` | 0.05 | частота `map → odom` (20 Гц). Было 0.01 — лишняя нагрузка |
| `map_update_interval` | 2.0 | как часто перерисовывать `/map`, с |
| `transform_timeout` | 0.2 | сколько ждать TF для скана |
| `tf_buffer_duration` | 30 | длина буфера TF, с |
| `throttle_scans` | 1 | брать каждый N-й скан |
| `enable_interactive_mode` | true | ручная правка графа из панели RViz |

### Когда добавлять узел

```yaml
use_scan_matching: true
minimum_travel_distance: 0.2    # было: 0.01 — тысячи шумных узлов
minimum_travel_heading: 0.2     # рад, было: 0.02
scan_buffer_size: 10
link_scan_maximum_distance: 1.5
```

Слишком частые узлы — это не «точнее», а наоборот: каждый узел вносит шум
сопоставления, граф раздувается, решатель тормозит. 0.2 м / 0.2 рад (≈11°) —
хороший баланс для скорости 0.3–0.5 м/с.

### Поиск совпадения скана

```yaml
correlation_search_space_dimension: 0.5    # было: 3.0
correlation_search_space_resolution: 0.01
correlation_search_space_smear_deviation: 0.1
coarse_search_angle_offset: 0.349          # ±20°
```

Размер окна поиска вокруг позы из одометрии. Было 3 м — поиск по сути
**игнорировал одометрию** и «прыгал» на похожие стены. 0.5 м достаточно,
если одометрия откалибрована (гл. 10). Если одометрия плохая и робот
«срывается» — увеличьте до 1.0, но сначала калибруйте.

### Замыкание петель

```yaml
do_loop_closing: true
loop_search_maximum_distance: 3.0       # было: 12 при дальности 2.5 м
loop_match_minimum_chain_size: 10
loop_match_minimum_response_coarse: 0.35
loop_match_minimum_response_fine: 0.45
loop_search_space_dimension: 8.0
```

| Симптом | Что крутить |
|---|---|
| Петля не замыкается, карта «двоится» | уменьшить `loop_match_minimum_response_*` (0.3/0.4), увеличить `loop_search_maximum_distance` |
| Ложные замыкания, карта «ломается» | увеличить `loop_match_minimum_response_*`, `loop_match_minimum_chain_size`; включить `HuberLoss` |

## 8.5. Режим локализации

`src/esp32_bridge/config/slam_localization.yaml` — набросок режима
`mode: localization` по сохранённой карте.

> **⚠ Внимание.** Файл не приведён в порядок: путь `map_file_name:
> "/root/ros2_ws/my_map"` от другой машины, `max_laser_range: 2.5` и
> `resolution: 0.02` расходятся с картой, построенной по `slam_param.yaml`.
> Перед использованием: укажите путь к своей карте (сериализованной через
> панель SlamToolboxPlugin → «Serialize Map»), выровняйте дальность и
> разрешение с режимом построения.

Запуск (через штатный launch локализации):

```bash
ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=src/esp32_bridge/config/slam_localization.yaml use_sim_time:=false
```

## 8.6. Сохранение карты

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map     # → map.pgm + map.yaml (для Nav2/map_server)
```

Или в RViz, панель **SlamToolboxPlugin**: «Save Map» (то же, `.pgm`) и
«Serialize Map» (граф целиком — для продолжения картографирования или
локализации slam_toolbox).

## 8.7. Порядок отладки SLAM

1. **TF.** `ros2 run tf2_ros tf2_echo map base_link` — есть ли цепочка.
2. **Время.** У всех нод одинаковый `use_sim_time`.
3. **Одометрия.** RViz, Fixed Frame = `odom`, `/scan` с Decay Time 10 с,
   медленно крутить робота: стены стоят на месте? Нет — SLAM не поможет,
   сначала гл. 5.9 и гл. 10.
4. **Скан.** `ros2 topic hz /scan` ≈ 5 Гц; в RViz скан не «рваный».
5. Только потом — параметры SLAM.

> **✎ Упражнение.** В симуляции поставьте `correlation_search_space_dimension:
> 3.0` и `minimum_travel_distance: 0.01`, объедьте остров дважды.
> Сохраните карту. Верните значения, повторите. Сравните две карты — это и
> есть причина, по которой появились комментарии «было:».
