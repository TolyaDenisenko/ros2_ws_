#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')

        # Подписываемся на данные одометрии
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Паблишер для топика с путем
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)

        # Инициализируем объект Path
        self.path_msg = Path()
        # Важно: frame_id должен совпадать с вашей системой координат (обычно "odom" или "map")
        self.path_msg.header.frame_id = 'odom'

    def odom_callback(self, msg: Odometry):
        # Создаем PoseStamped из данных одометрии
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Добавляем новую позу в список
        self.path_msg.poses.append(pose_stamped)

        # Обновляем время в Path-сообщении (важно для корректного отображения)
        self.path_msg.header.stamp = msg.header.stamp

        # Публикуем обновлённый путь
        self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
