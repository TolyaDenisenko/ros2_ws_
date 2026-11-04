"""Nav2 + octomap поверх уже запущенного SLAM (slam_toolbox даёт /map и map→odom).

Запуск (симуляция):
  ros2 launch hoverbot_sim mapping.launch.py          # терминал 1
  ros2 launch hoverbot_nav navigation.launch.py       # терминал 2
Реальный робот:
  ros2 launch esp32_bridge robot.launch.py transport:=udp host:=<IP>
  ros2 launch hoverbot_nav navigation.launch.py use_sim_time:=false
Цель: кнопка «Nav2 Goal» в RViz (топик /goal_pose).

Аргументы:
  octomap:=false             не запускать octomap_server
  use_octomap_costmap:=true  статический слой global costmap берёт /projected_map
                             (проекция octomap) вместо /map slam_toolbox

Нужны пакеты: ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-octomap-server
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('hoverbot_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap = LaunchConfiguration('octomap')
    use_octomap_costmap = LaunchConfiguration('use_octomap_costmap')

    # use_sim_time во все ноды + выбор карты для статического слоя
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
                PythonExpression(["'/projected_map' if '", use_octomap_costmap,
                                  "' == 'true' else '/map'"]),
        },
        convert_types=True,
    )

    lifecycle_nodes = ['controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'velocity_smoother']

    def nav_node(package, executable, name, remaps=()):
        return Node(package=package, executable=executable, name=name, output='screen',
                    # use_sim_time отдельным словарём: в yaml такого ключа нет, RewrittenYaml его не добавит
                    parameters=[params, {'use_sim_time': use_sim_time}], remappings=list(remaps))

    # controller → cmd_vel_nav → velocity_smoother → cmd_vel
    nav_nodes = [
        nav_node('nav2_controller', 'controller_server', 'controller_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': lifecycle_nodes}])

    # Octomap: /scan → облако → octomap_server (3D-вокселы в кадре map)
    scan_to_cloud = Node(
        package='hoverbot_nav', executable='scan_to_cloud', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(octomap))

    octomap_server = Node(
        package='octomap_server', executable='octomap_server_node', name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'resolution': 0.05,
            'latch': True,   # transient_local: static_layer costmap подписывается так же
            'sensor_model.max_range': 6.0,
            'filter_ground_plane': False,
            # Лидар 2D: проецируем в OccupancyGrid всё выше пола в пределах корпуса
            'occupancy_min_z': -0.1,
            'occupancy_max_z': 0.6,
        }],
        remappings=[('cloud_in', 'scan_cloud')],
        condition=IfCondition(octomap))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Время симуляции (false на реальном роботе)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
                              description='YAML с параметрами Nav2'),
        DeclareLaunchArgument('octomap', default_value='true',
                              description='Запустить octomap_server'),
        DeclareLaunchArgument('use_octomap_costmap', default_value='false',
                              description='global costmap по /projected_map вместо /map'),
        *nav_nodes,
        # Менеджер стартует позже нод: иначе его первый вызов configure приходит,
        # пока сервисы controller_server ещё не обнаружены, и bringup обрывается
        TimerAction(period=4.0, actions=[lifecycle_manager]),
        scan_to_cloud,
        octomap_server,
    ])
