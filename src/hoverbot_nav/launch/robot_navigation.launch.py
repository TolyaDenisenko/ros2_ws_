"""РЕАЛЬНЫЙ робот: esp32_bridge + slam_toolbox + Nav2 + octomap.

ВНИМАНИЕ: робот поедет сам. Первый запуск — на подставке (колёса в воздухе)
или с рукой на выключателе; чек-лист — docs/CHECKLIST.md, раздел «Железо».

Запуск:
  ros2 launch hoverbot_nav robot_navigation.launch.py transport:=udp host:=192.168.0.105
  ros2 launch hoverbot_nav robot_navigation.launch.py transport:=serial device:=/dev/ttyACM0
Аргументы:
  rviz:=false     без RViz (бортовой ПК без экрана; RViz запускают на другом ПК)
  octomap:=false  без octomap_server
  use_octomap_costmap:=true  global costmap по /projected_map
Всё идёт на реальном времени: use_sim_time=false (у slam_toolbox по умолчанию true!).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge_share = get_package_share_directory('esp32_bridge')
    nav_share = get_package_share_directory('hoverbot_nav')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bridge_share, 'launch', 'robot.launch.py')),
        launch_arguments={'transport': LaunchConfiguration('transport'),
                          'host': LaunchConfiguration('host'),
                          'device': LaunchConfiguration('device'),
                          'slam': 'true',
                          'rviz': LaunchConfiguration('rviz'),
                          'rviz_config': os.path.join(nav_share, 'rviz', 'navigation.rviz')}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'octomap': LaunchConfiguration('octomap'),
                          'use_octomap_costmap': LaunchConfiguration('use_octomap_costmap')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='udp', description='udp (Wi-Fi) или serial (USB)'),
        DeclareLaunchArgument('host', default_value='192.168.0.105', description='IP ESP32 (udp)'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0', description='Порт ESP32 (serial)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz'),
        DeclareLaunchArgument('octomap', default_value='true', description='octomap_server'),
        DeclareLaunchArgument('use_octomap_costmap', default_value='false',
                              description='global costmap по /projected_map вместо /map'),
        robot,
        # Даём мосту и SLAM поднять /scan, /odom и TF map→odom
        TimerAction(period=8.0, actions=[nav]),
    ])
