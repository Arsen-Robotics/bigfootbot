import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class RelayControlNode(Node):
    def __init__(self):
        super().__init__('relay_control_node')

        self.serial_port = serial.Serial('/dev/cr6a-arduino-mega', baudrate=9600, timeout=1)
        time.sleep(2)

        self.subscription = self.create_subscription(
            String,
            'relay_command',
            self.command_callback,
            10)

        def command_callback(self, msg):
            command = msg.data
            self.send_command_to_serial(command)

        def send_command_to_serial(self, command):
            self.serial_port.write(command.encode())

def main():
    rclpy.init()
    relay_control_node = RelayControlNode()
    rclpy.spin(relay_control_node)
    relay_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()