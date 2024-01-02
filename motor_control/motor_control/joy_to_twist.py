import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')

        time.sleep(2)

        self.linear_scale = 1.59
        self.angular_scale = 7.0

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.command_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

    def command_callback(self, msg):
        twist_msg = Twist()

        twist_msg.linear.x = self.linear_scale * msg.axes[1]
        twist_msg.angular.z = self.angular_scale * msg.axes[3]

        self.publisher.publish(twist_msg)

def main():
    rclpy.init()
    joy_to_twist_node = JoyToTwistNode()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
