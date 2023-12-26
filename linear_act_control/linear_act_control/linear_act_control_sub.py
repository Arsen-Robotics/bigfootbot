import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class LinearActControlNode(Node):
    def __init__(self):
        super().__init__('linear_act_control_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        self.command = 's'
        
        self.subscription = self.create_subscription(
            String,
            'linear_act_command',
            self.command_callback,
            10)
   
    def command_callback(self, msg):
        command = msg.data
        self.send_command_to_serial(command)

    def send_command_to_serial(self, command):
        self.serial_port.write(command.encode())
        time.sleep(0.1)  # Optional: add a small delay to ensure proper communication

def main():
    rclpy.init()
    linear_act_control_node = LinearActControlNode()
    rclpy.spin(linear_act_control_node)
    linear_act_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
