import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')

        self.enable_button = 4
        self.linear_axis = 1
        self.angular_axis = 3 # 2 for Foxglove joystick extension
        self.linear_scale = 3.067
        self.angular_scale = 9.437

        # Create a subscription to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.command_callback,
            10)

        # Create a publisher that will publish Twist messages to the cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

    # This function is called every time a message is received on the joy topic
    # It reads joy messages and converts them to Twist messages
    # and publishes them to the cmd_vel topic
    def command_callback(self, msg):
        try:
            if msg.buttons[self.enable_button] == 1: # Check if enable button is pressed
                twist_msg = Twist()

                twist_msg.linear.x = self.linear_scale * msg.axes[self.linear_axis]
                twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]

                self.publisher.publish(twist_msg)

        # If an exception occurs, print the exception to the console
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main():
    rclpy.init()
    joy_to_twist_node = JoyToTwistNode()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
