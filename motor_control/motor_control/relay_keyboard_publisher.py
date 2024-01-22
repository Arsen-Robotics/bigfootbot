import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard

class RelayKeyboardPublisherNode(Node):
    def __init__(self):
        super().__init__('relay_keyboard_publisher_node')

        self.publisher = self.create_publisher(
            String,
            'relay_command',
            10)