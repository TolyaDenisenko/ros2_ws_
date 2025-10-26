import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from rclpy.time import Time

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.twist_callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

    def twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist = msg  # Копируем данные Twist
        self.publisher.publish(twist_stamped)
def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()