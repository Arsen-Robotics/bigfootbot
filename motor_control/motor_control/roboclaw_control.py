import rclpy
from rclpy.node import Node
import serial
from roboclaw import Roboclaw
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

        serial_obj = serial.Serial('/dev/roboclaw', 38400) # default baudrate is 38400
        self.rclaw = Roboclaw(serial_obj)

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

        if left_motor_command < 0:
            self.rclaw.backward_m1(abs(left_motor_command))

        if right_motor_command < 0:
            self.rclaw.backward_m2(abs(left_motor_command))

        if left_motor_command >= 0:
            self.rclaw.forward_m1(left_motor_command)

        if right_motor_command >= 0:
            self.rclaw.forward_m2(right_motor_command)

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

        self.get_logger().info(f"{left_motor_command} {right_motor_command} {left_motor_speed_rpm} {right_motor_speed_rpm}")

        return left_motor_command, right_motor_command
    
def main():
    rclpy.init()
    roboclaw_control_node = RoboclawControlNode()
    rclpy.spin(roboclaw_control_node)
    roboclaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()