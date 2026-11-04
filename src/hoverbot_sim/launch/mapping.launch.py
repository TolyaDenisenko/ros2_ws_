"""Всё сразу: Gazebo + робот + RViz + построение карты (slam_toolbox).

Запуск:
  ros2 launch hoverbot_sim mapping.launch.py
  ros2 launch hoverbot_sim mapping.launch.py gui:=false   # без окна Gazebo
Управление (в отдельном терминале):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Сохранить карту:
  ros2 run nav2_map_server map_saver_cli -f ~/map
  (или кнопка «Save Map» в панели SlamToolboxPlugin в RViz)

Нужен пакет slam_toolbox:  sudo apt install ros-$ROS_DISTRO-slam-toolbox
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('hoverbot_sim')

    slam_params = LaunchConfiguration('slam_params')

    # Симуляция (Gazebo, робот, мост, RViz) — с RViz-конфигом в кадре map
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    # SLAM: подписывается на /scan и TF odom→base_link,
    # публикует /map и TF map→odom.
    # Начиная с Jazzy slam_toolbox — lifecycle-нода: сама по себе она стоит
    # в состоянии unconfigured и ничего не делает. Штатный launch-файл
    # slam_toolbox переводит её в active (autostart), поэтому подключаем его.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Окно Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Запустить RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'mapping.rviz'),
            description='Файл конфигурации RViz'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(pkg_share, 'config', 'slam_sim.yaml'),
            description='YAML с параметрами slam_toolbox'),
        sim,
        slam,
    ])
