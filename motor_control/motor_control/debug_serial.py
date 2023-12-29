import rclpy
from rclpy.node import Node
import serial
import time

class DebugSerialNode(Node):
    def __init__(self):
        super().__init__('debug_serial_node')
        self.serial_port = serial.Serial('/dev/cr6a-arduino-mega', baudrate=9600, timeout=1)

        time.sleep(2)
        self.publish_message()

    def publish_message(self):
        m1_speed = 0
        m2_speed = 0
        acceleration = 15 # Adjust acceleration as needed
        max_speed = 255
        min_speed = -255

        # Gradually increase speeds
        for speed in range(0, max_speed + 1, acceleration):
            m1_speed = speed
            m2_speed = speed
            command = f"m {-m1_speed} {m2_speed}\r"
            self.serial_port.write(command.encode())
            print(f"Sent command: {command}")
            time.sleep(0.05)

        # Instant stop
        m1_speed = 0
        m2_speed = 0
        command = f"m {-m1_speed} {m2_speed}\r"
        self.serial_port.write(command.encode())
        print(f"Sent command: {command}")
        time.sleep(0.2)

        # Gradually decrease speeds
        for speed in range(0, min_speed - 1, -acceleration):
            m1_speed = speed
            m2_speed = speed
            command = f"m {-m1_speed} {m2_speed}\r"
            self.serial_port.write(command.encode())
            print(f"Sent command: {command}")
            time.sleep(0.05)

        # Instant stop
        m1_speed = 0
        m2_speed = 0
        command = f"m {-m1_speed} {m2_speed}\r"
        self.serial_port.write(command.encode())
        print(f"Sent command: {command}")
        time.sleep(0.2)

    def destroy_node(self):
    # Close the serial port when the node is shutting down
    # Add code here to close the serial port
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')

        super().destroy_node()

def main():
    rclpy.init()
    debug_serial_node = DebugSerialNode()
    rclpy.spin(debug_serial_node)
    debug_serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()