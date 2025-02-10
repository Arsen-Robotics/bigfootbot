#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

# This node subscribes to the 'cmd_vel_unstamped' topic of type geometry_msgs/msg/Twist
# converts message to geometry_msgs/msg/TwistStamped and republishes it to the 'cmd_vel' topic
class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        
        # Create subscriber for Twist messages
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_unstamped',  # Original topic
            self.twist_callback,
            10)
        
        # Create publisher for TwistStamped messages
        self.publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel',  # Target topic for diff_drive_controller
            10)

    def twist_callback(self, twist_msg):
        # Convert Twist to TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'  # or whatever frame you want
        stamped_msg.twist = twist_msg
        
        # Publish the stamped message
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 