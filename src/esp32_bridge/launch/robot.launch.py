"""Реальный робот: связь с ESP32 + построение карты (slam_toolbox) + RViz.

Запуск:
  ros2 launch esp32_bridge robot.launch.py transport:=udp host:=192.168.0.105   # Wi-Fi
  ros2 launch esp32_bridge robot.launch.py transport:=serial                    # USB, /dev/ttyACM0
Аргументы:
  slam:=false  — только связь с ESP32, без построения карты
  rviz:=false  — без RViz (например, на бортовом ПК без экрана)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('esp32_bridge')

    bridge = Node(
        package='esp32_bridge',
        executable='esp32_bridge',
        output='screen',
        parameters=[{
            'transport': LaunchConfiguration('transport'),
            'host': LaunchConfiguration('host'),
            'device': LaunchConfiguration('device'),
        }],
    )

    # slam_toolbox — lifecycle-нода, поэтому через его штатный launch-файл
    # (он её активирует). use_sim_time:=false обязательно: по умолчанию там true.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_param.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='udp', description='udp (Wi-Fi) или serial (USB)'),
        DeclareLaunchArgument('host', default_value='192.168.0.105', description='IP ESP32 (udp)'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0', description='Порт ESP32 (serial)'),
        DeclareLaunchArgument('slam', default_value='true', description='Строить карту'),
        DeclareLaunchArgument('rviz', default_value='true', description='Запустить RViz'),
        bridge,
        slam,
        rviz,
    ])
