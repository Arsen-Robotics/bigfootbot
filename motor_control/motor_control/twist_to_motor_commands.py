import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import math

class TwistToMotorCommandsNode(Node):
    def __init__(self):
        super().__init__('twist_to_motor_commands_node')
        self.serial_port = serial.Serial('/dev/cr6a-arduino-mega', baudrate=9600, timeout=1)

        time.sleep(2)

        self.mode = 'm'
        self.left_motor_command = 0
        self.right_motor_command = 0

        self.wheel_base = 0.335 # Distance between left and right wheel in meters (0.31 for ARS-CAR)
        self.wheel_diameter = 0.1 # Wheel diameter in meters (0.06 for ARS-CAR)

        self.max_rpm = 303 # Motor's maximum RPM (445 for ARS-CAR)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10) # 10 is the queue size (how many messages to store in memory)

    def command_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Calculate the linear speed (in m/s) of each wheel
        left_wheel_linear_speed = linear_speed - (self.wheel_base / 2.0) * angular_speed
        right_wheel_linear_speed = linear_speed + (self.wheel_base / 2.0) * angular_speed

        #self.get_logger().info(f"Linear speed L: {left_wheel_linear_speed}")
        #self.get_logger().info(f"Linear speed R: {right_wheel_linear_speed}")

        # Calculate the left and right motor speeds in revolutions per minute (RPM)
        left_motor_speed_rpm = left_wheel_linear_speed / (math.pi * self.wheel_diameter) * 60
        right_motor_speed_rpm = right_wheel_linear_speed / (math.pi * self.wheel_diameter) * 60

        #self.get_logger().info(f"RPM L: {left_motor_speed_rpm}")
        #self.get_logger().info(f"RPM R: {right_motor_speed_rpm}")

        # Convert RPM to motor commands (-255 to +255)
        self.left_motor_command = int(left_motor_speed_rpm / self.max_rpm * 255)
        self.right_motor_command = int(right_motor_speed_rpm / self.max_rpm * 255)

        #self.get_logger().info(f"Left Motor Command Convert: {self.left_motor_command}")
        #self.get_logger().info(f"Right Motor Command Convert: {self.right_motor_command}")

        if self.left_motor_command != self.right_motor_command:
            if self.left_motor_command > 255 or self.right_motor_command > 255:
                difference = \
                max(self.left_motor_command, self.right_motor_command) - 255

                if self.left_motor_command > self.right_motor_command:
                    self.right_motor_command = self.right_motor_command - difference

                elif self.right_motor_command > self.left_motor_command:
                    self.left_motor_command = self.left_motor_command - difference
            
        # Ensure the motor commands are within the valid range (-255 to +255)
        self.left_motor_command = max(min(self.left_motor_command, 255), -255)
        self.right_motor_command = max(min(self.right_motor_command, 255), -255)

        #self.get_logger().info(f"Left Motor Command V Range: {self.left_motor_command}")
        #self.get_logger().info(f"Right Motor Command V Range: {self.right_motor_command}")
         
        if self.left_motor_command == 0 and self.right_motor_command == 0:
            self.mode = 's' # Electric brake mode
        else: self.mode = 'm' # PWM mode

        # Construct a string to send to the Arduino in the following format:
        # "<mode> <left motor speed> <right motor speed> \r" (e.g. "m 100 100\r")
        data = f"{self.mode} {-self.left_motor_command} {self.right_motor_command}\r"

        self.send_command_to_serial(data)

    def send_command_to_serial(self, data):
        self.serial_port.write(data.encode())
        #time.sleep(0.1)  # Optional: add a small delay to ensure proper communication

    def destroy_node(self):
        # Close the serial port when the node is shutting down
        # Add code here to close the serial port
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')

        super().destroy_node()

def main():
    rclpy.init()
    twist_to_motor_commands_node = TwistToMotorCommandsNode()
    rclpy.spin(twist_to_motor_commands_node)
    twist_to_motor_commands_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
