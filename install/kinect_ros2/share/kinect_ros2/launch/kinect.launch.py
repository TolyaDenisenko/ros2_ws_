from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Launching Kinect v1 component..."),

        ComposableNodeContainer(
            name='kinect_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='kinect_ros2',
                    plugin='kinect_ros2::KinectRosComponent',
                    name='kinect_v1',
                    # Параметры можно добавить здесь, если понадобятся:
                    # parameters=[{'use_sim_time': False}]
                )
            ],
            output='screen',
            emulate_tty=True,  # Для корректного отображения цветов и логов
        ),
    ])
