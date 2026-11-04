"""Симуляция тележки в Gazebo Sim: мир-комната, робот, мост в ROS 2 и RViz.

Запуск:
  ros2 launch hoverbot_sim sim.launch.py               # с GUI Gazebo и RViz
  ros2 launch hoverbot_sim sim.launch.py gui:=false    # без окна Gazebo
Управление:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hoverbot_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hoverbot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'room.sdf')
    bridge_file = os.path.join(pkg_share, 'config', 'bridge.yaml')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Модель робота: xacro разворачивается в URDF при запуске,
    # поэтому правки размеров в .xacro подхватываются без пересборки
    # (при colcon build --symlink-install)

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Gazebo: -r — сразу запустить физику, -s — только сервер (без окна)
    gz_args = PythonExpression([
        "'-r ", world_file, "' + ('' if '", gui, "' == 'true' else ' -s')"
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    # Публикует TF корпус→колёса/лидар из URDF и топик /robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Ставим робота у западной стены, между стеной и островом
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'hoverbot',
                   '-x', '-2.5', '-y', '0.0', '-z', '0.1'],
    )

    # Мост топиков Gazebo ↔ ROS 2 (список — в config/bridge.yaml)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Окно Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Запустить RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'sim.rviz'),
            description='Файл конфигурации RViz'),
        gazebo,
        rsp,
        spawn,
        bridge,
        rviz_node,
    ])
