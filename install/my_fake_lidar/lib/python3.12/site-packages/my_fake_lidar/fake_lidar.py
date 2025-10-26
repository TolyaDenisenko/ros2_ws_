#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')

        # Паблишер LaserScan
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Таймер для периодической публикации (10 Гц)
        self.timer = self.create_timer(0.1, self.publish_scan)

        # Параметры лидара
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180.0  # 1 градус между лучами
        self.range_min = 0.2
        self.range_max = 10.0

        # Вычисляем количество лучей
        self.num_readings = int(abs(self.angle_max - self.angle_min) / self.angle_increment)

    def publish_scan(self):
        scan_msg = LaserScan()
        now = self.get_clock().now().to_msg()

        scan_msg.header.stamp = now
        # Здесь frame_id должен совпадать с тем, что описано в URDF (обычно это lidar_link)
        scan_msg.header.frame_id = 'lidar_link'

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # т.к. публикуем каждые 0.1 сек
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Сгенерируем какие-нибудь «фиктивные» расстояния (например, все на 2.0 м)
        ranges = [2.0 for _ in range(self.num_readings)]
        # Можно добавить лёгкий разброс для имитации «шума»
        import random
        ranges = [2.0 + random.uniform(-0.1, 0.1) for _ in range(self.num_readings)]

        scan_msg.ranges = ranges

        # Интенсивности (если не нужны, можно оставить пустыми)
        scan_msg.intensities = [0.0] * self.num_readings

        self.scan_publisher.publish(scan_msg)
        self.get_logger().info('Published fake LaserScan')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
