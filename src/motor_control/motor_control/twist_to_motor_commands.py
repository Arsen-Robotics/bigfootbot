import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import math
import statistics

class TwistToMotorCommandsNode(Node):
    def __init__(self):
        super().__init__('twist_to_motor_commands_node')

        self.wheel_base = 0.335 # Distance between left and right wheel in meters (0.31 for ARS-CAR)
        self.wheel_diameter = 0.1 # Wheel diameter in meters (0.06 for ARS-CAR)
        self.max_rpm = 303 # Motor's maximum RPM (445 for ARS-CAR)

        # Open the serial port
        self.serial_port = serial.Serial('/dev/cr6a-arduino-mega', baudrate=9600, timeout=1)
        time.sleep(2)

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10) # 10 is the queue size (how many messages to store in memory)

        #self.timer = self.create_timer(0.01, self.get_ultrasonic_sensor_distance)
        #self.get_ultrasonic_sensor_distance()

        self.distance_to_obs = 0 # Distance to the obstacle in centimeters as integer (complete)
        self.distance_to_obs_incomplete = '' # Distance to the obstacle in centimeters as string (incomplete)
        self.ident = '' # Ultrasonic sensor identifier, that comes before the distance (e.g. "d 25\r")
        self.index = 0 # Which argument is being processed (0 = ident, 1 = distance)

        self.danger_distance = 65 # Distance to obstacle in cm when robot should start braking
        self.stopped_due_to_obstacle = 0 # 1 = robot has stopped due to obstacle, 0 = robot is moving
        self.last_distance_to_obs = 0 # Distance to obstacle in cm when robot last time stopped

    def command_callback(self, msg):
        # Unpack the tuple returned by twist_to_motor_commands function into two variables
        # left_motor_command and right_motor_command [-255, 255]
        left_motor_command, right_motor_command = self.twist_to_motor_commands(msg)

        '''if self.distance_to_obs > self.last_distance_to_obs + 10: # 10 cm
            self.stopped_due_to_obstacle = 0

        if self.distance_to_obs < self.danger_distance\
        and self.stopped_due_to_obstacle == 0\
        and (left_motor_command > 0 and right_motor_command > 0):
            # Define the range of motor speed and stopping distance
            speed_range = (0, 255)
            distance_range = (10, 60)
            speed = (left_motor_command + right_motor_command) / 2

            # Perform linear interpolation
            stopping_distance = distance_range[0] + (speed - speed_range[0]) * (
                distance_range[1] - distance_range[0]) / (speed_range[1] - speed_range[0])

            if self.distance_to_obs < stopping_distance:
                self.serial_port.write(f"m {-0.5*speed} {-0.5*speed}\r".encode())
                self.stopped_due_to_obstacle = 1
                time.sleep(0.2)
                self.serial_port.write(f"s\r".encode())
                self.last_distance_to_obs = self.distance_to_obs'''

        # Set the mode to 's' (electrical braking) if the linear and angular speed are both 0
        if left_motor_command == 0 and right_motor_command == 0:
            mode = 's' # Electric brake mode
        else: mode = 'm' # PWM mode

        # Construct a string to send to the Arduino in the following format:
        # "<mode> <left motor speed> <right motor speed> \r" (e.g. "m 100 100\r")
        if self.stopped_due_to_obstacle == 0\
        or (left_motor_command < 0 and right_motor_command < 0):
            data = f"{mode} {left_motor_command} {right_motor_command}\r"
            self.serial_port.write(data.encode())  # Send the data to the serial port

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

        # Convert RPM to motor commands (-255 to +255)
        left_motor_command = int(left_motor_speed_rpm / self.max_rpm * 255)
        right_motor_command = int(right_motor_speed_rpm / self.max_rpm * 255)

        # If some of the commands are over 255 or under -255,
        # modify them to maintain the ratio between the left and right motor commands
        # For example, Left = 300, Right = 200, after modification: Left = 255, Right = 155
        if left_motor_command != right_motor_command:
            if left_motor_command > 255 or right_motor_command > 255:
                difference = \
                max(left_motor_command, right_motor_command) - 255

                if left_motor_command > right_motor_command:
                    right_motor_command = right_motor_command - difference

                elif right_motor_command > left_motor_command:
                    left_motor_command = left_motor_command - difference

            if left_motor_command < -255 or right_motor_command < -255:
                difference = \
                min(left_motor_command, right_motor_command) + 255

                if left_motor_command > right_motor_command:
                    left_motor_command = left_motor_command - difference

                if right_motor_command > left_motor_command:
                    right_motor_command = right_motor_command - difference
            
        # Ensure the motor commands are within the valid range (-255 to +255)
        left_motor_command = max(min(left_motor_command, 255), -255)
        right_motor_command = max(min(right_motor_command, 255), -255)

        return left_motor_command, right_motor_command

    def get_ultrasonic_sensor_distance(self):
        byte = self.serial_port.read(1) # Read one byte from the serial port
        
        if byte:
            #self.get_logger().info(f"distance count: {self.distance_count}")
            #self.get_logger().info(f"byte: {byte}")
            # Convert the byte to a char
            char = byte.decode('utf-8')
                
            # If char is carriage return (\r, ASCII 13), then the data is complete
            if char == '\r':
                # End of data, convert distance to an integer and set it to self.distance_to_obs
                if self.distance_to_obs_incomplete:
                    self.distance_to_obs = int(self.distance_to_obs_incomplete)

                    # Clear the input buffer to prevent buffer overflow
                    self.serial_port.flushInput()

                    # Reset the index, ident and distance variables
                    self.index = 0
                    self.ident = ''
                    self.distance_to_obs_incomplete = ''

            elif char == ' ' and self.ident == 'd':
                # Space indicates the transition from ident to distance (e.g "d 25\r")
                self.index += 1
            # If index is 0 (ident) and char is a letter, set ident to char (start of data)
            elif self.index == 0 and char.isalpha():
                self.ident = char
            # If index is 1 (distance) and char is a digit, append char to distance
            elif self.index == 1 and char.isdigit():
                self.distance_to_obs_incomplete += char

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