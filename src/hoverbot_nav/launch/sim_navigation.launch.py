"""Симуляция целиком: Gazebo + SLAM + Nav2 + octomap одной командой.

Запуск:
  ros2 launch hoverbot_nav sim_navigation.launch.py [gui:=false] [rviz:=false]
Цель — «Nav2 Goal» в RViz или:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    sim_share = get_package_share_directory('hoverbot_sim')
    nav_share = get_package_share_directory('hoverbot_nav')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'mapping.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz'),
                          'rviz_config': os.path.join(nav_share, 'rviz', 'navigation.rviz')}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'octomap': LaunchConfiguration('octomap'),
                          'use_octomap_costmap': LaunchConfiguration('use_octomap_costmap')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Окно Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz'),
        DeclareLaunchArgument('octomap', default_value='true', description='octomap_server'),
        DeclareLaunchArgument('use_octomap_costmap', default_value='false',
                              description='global costmap по /projected_map вместо /map'),
        sim,
        # Nav2 стартует после появления TF odom→base_link и /map (Gazebo грузится ~10 с)
        TimerAction(period=10.0, actions=[nav]),
    ])
