# Official RoboClaw Python library installation:
# Download the library: http://downloads.basicmicro.com/code/roboclaw_python.zip
# Unzip the library, move roboclaw_python folder to ~/.local/lib/python3.10/site-packages

import rclpy
from rclpy.node import Node
from roboclaw_python.roboclaw_3 import Roboclaw
from geometry_msgs.msg import Twist
import math
import time

class RoboclawControlNode(Node):
    def __init__(self):
        super().__init__('roboclaw_control_node')

        self.wheel_base = 0.65 # Distance between left and right wheel in meters
        self.wheel_diameter = 0.33 # Wheel diameter in meters
        self.max_rpm = 177.5
        self.max_motor_command = 126

        self.rclaw = Roboclaw("/dev/roboclaw", 9600)
        self.address = 0x80
        self.rclaw.Open()

        time.sleep(2)

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10) # 10 is the queue size (how many messages to store in memory)
        
    def command_callback(self, msg):
        # Unpack the tuple returned by twist_to_motor_commands function into two variables
        # left_motor_command and right_motor_command [-127, 127]
        left_motor_command, right_motor_command = self.twist_to_motor_commands(msg)

        self.get_logger().info(f"{left_motor_command} {right_motor_command}")

        if left_motor_command < 0:
            self.rclaw.BackwardM1(self.address, abs(left_motor_command))

        if right_motor_command < 0:
            self.rclaw.BackwardM2(self.address, abs(right_motor_command))

        if left_motor_command >= 0:
            self.rclaw.ForwardM1(self.address, left_motor_command)

        if right_motor_command >= 0:
            self.rclaw.ForwardM2(self.address, right_motor_command)

    # Convert a Twist message to motor commands and return them as a tuple (left, right)
    def twist_to_motor_commands(self, msg):
        # Get the linear and angular speed from the Twist message
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Calculate the linear speed (in m/s) of each wheel
        left_wheel_linear_speed = linear_speed - (self.wheel_base / 2.0) * angular_speed
        right_wheel_linear_speed = linear_speed + (self.wheel_base / 2.0) * angular_speed

        # Calculate the left and right motor speeds in revolutions per minute (RPM)
        left_motor_speed_rpm = left_wheel_linear_speed / (math.pi * self.wheel_diameter) * 60
        right_motor_speed_rpm = right_wheel_linear_speed / (math.pi * self.wheel_diameter) * 60

        # Convert RPM to motor commands
        left_motor_command = int(left_motor_speed_rpm / self.max_rpm * self.max_motor_command)
        right_motor_command = int(right_motor_speed_rpm / self.max_rpm * self.max_motor_command)

        # If some of the commands are over 127 or under -127,
        # modify them to maintain the ratio between the left and right motor commands
        if left_motor_command != right_motor_command:
            if left_motor_command > self.max_motor_command or right_motor_command > self.max_motor_command:
                difference = \
                max(left_motor_command, right_motor_command) - self.max_motor_command

                if left_motor_command > right_motor_command:
                    right_motor_command = right_motor_command - difference

                elif right_motor_command > left_motor_command:
                    left_motor_command = left_motor_command - difference

            if left_motor_command < -self.max_motor_command or right_motor_command < -self.max_motor_command:
                difference = \
                min(left_motor_command, right_motor_command) + self.max_motor_command

                if left_motor_command > right_motor_command:
                    left_motor_command = left_motor_command - difference

                if right_motor_command > left_motor_command:
                    right_motor_command = right_motor_command - difference
            
        # Ensure the motor commands are within the valid range (-127 to +127)
        left_motor_command = max(min(left_motor_command, self.max_motor_command), -self.max_motor_command)
        right_motor_command = max(min(right_motor_command, self.max_motor_command), -self.max_motor_command)

        return left_motor_command, right_motor_command
    
def main():
    rclpy.init()
    roboclaw_control_node = RoboclawControlNode()
    rclpy.spin(roboclaw_control_node)
    roboclaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()