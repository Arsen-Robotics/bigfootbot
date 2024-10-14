# Official RoboClaw Python library installation:
# Download the library: http://downloads.basicmicro.com/code/roboclaw_python.zip
# Unzip the library, move roboclaw_python folder to ~/.local/lib/python3.*/site-packages

import rclpy
from rclpy.node import Node
# from roboclaw_python.roboclaw_3 import Roboclaw
from new_roboclaw_driver.roboclaw import Roboclaw
from geometry_msgs.msg import Twist
import math
from bfb_interfaces.msg import RoboclawState
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

class RoboclawControlNode(Node):
    def __init__(self):
        super().__init__('roboclaw_control_node')

        self.wheel_track = 0.65 # Distance between left and right wheel in meters
        self.wheel_diameter = 0.33 # Wheel diameter in meters
        self.max_rpm = 177.5
        self.max_motor_command = 126

        self.comport = "/dev/roboclaw"
        self.baudrate = 38400

        self.rclaw = Roboclaw(self.comport, self.baudrate)

        # Address is set in Basicmicro Motion Studio to determine which Roboclaw to talk to,
        # if multiple Roboclaws are connected to the same serial port
        self.address = 0x80

        # Initially this flag is set to None, because it is not known if the Roboclaw is connected or not
        self.rclaw_connected = None

        self.turn_compensation_factor = 0.5

        self.max_battery_voltage = 29.4
        self.min_battery_voltage = 23

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10) # 10 is the queue size (how many messages to store in memory)

        # Create a publisher that will publish Roboclaw state as RoboclawState message
        self.roboclaw_state_publisher = self.create_publisher(
            RoboclawState,
            'roboclaw_state',
            10)

        # Create a publisher that will publish battery state as BatteryState message
        self.battery_state_publisher = self.create_publisher(
            BatteryState,
            'battery_state',
            10)

        self.wheel_speed_publisher = self.create_publisher(
            Float32,
            'wheel_speed',
            10)

        # Timer will call publish_roboclaw_state function every 0.6 sec
        self.timer = self.create_timer(0.6, self.publish_roboclaw_state)

        # self.test_timer = self.create_timer(0.05, self.command_callback)

    # This function tries to open a serial connection to the Roboclaw
    # If the connection succeeds or fails, it prints a message to the console
    # The flag self.rclaw_connected is only for the reason
    # that the message is printed only once when the connection is established or lost
    # The function returns the value of self.rclaw_connected
    def connect_to_roboclaw(self):
        if self.rclaw.open():
            if self.rclaw_connected == False or self.rclaw_connected == None:
                self.rclaw_connected = True
                self.get_logger().info("Roboclaw connected")

        else:
            if self.rclaw_connected == True or self.rclaw_connected == None:
                self.rclaw_connected = False
                self.get_logger().error("Failed to open Roboclaw, retrying...")

        return self.rclaw_connected

    def publish_roboclaw_state(self):
        try:
            # If the Roboclaw is not connected, exit the function to avoid errors when calling Roboclaw
            if not self.connect_to_roboclaw():
                return

            roboclaw_state = RoboclawState()

            # Currents
            # currents = self.rclaw.ReadCurrents(self.address)
            currents = self.rclaw.read_motor_currents(self.address)

            # current1_val = currents[1] / 100
            # current2_val = currents[2] / 100
            current1_val = currents[0] / 100
            current2_val = currents[1] / 100

            roboclaw_state.current_1 = current1_val
            roboclaw_state.current_2 = current2_val

            self.get_logger().info(f"{roboclaw_state.current_1} {roboclaw_state.current_2}")

            # Main battery voltage
            # main_battery_voltage_val = self.rclaw.ReadMainBatteryVoltage(self.address)
            main_battery_voltage_val = self.rclaw.read_main_battery_voltage(self.address)
            # roboclaw_state.main_battery_voltage = main_battery_voltage_val[1] / 10
            roboclaw_state.main_battery_voltage = main_battery_voltage_val / 10

            self.publish_battery_state(roboclaw_state.main_battery_voltage)

            self.get_logger().info(f"{roboclaw_state.main_battery_voltage}")

            # Temperature
            # temp1_val = self.rclaw.ReadTemp(self.address)
            temp1_val = self.rclaw.read_temperature(self.address)
            # temp2_val = self.rclaw.ReadTemp2(self.address)

            # roboclaw_state.temp1 = temp1_val[1] / 10
            roboclaw_state.temp1 = temp1_val / 10
            # roboclaw_state.temp2 = temp2_val[1] / 10

            self.get_logger().info(f"{roboclaw_state.temp1}")

            # Publish roboclaw state
            self.roboclaw_state_publisher.publish(roboclaw_state)

        # Even though the connection is checked in the connect_to_roboclaw function,
        # the connection can be lost while program is communicating with Roboclaw,
        # so OSError exception (usually because of Input/Output error) is caught here
        except OSError:
            self.rclaw_connected = False
            self.get_logger().error("Failed to open Roboclaw, retrying...")
        
        # All known exceptions are caught, but if some unknown exception occurs,
        # it is caught here and printed to the console
        except Exception as e:
            self.get_logger().error(f"Exception 1: {e}")

    def publish_battery_state(self, voltage_val):
        battery_state = BatteryState()

        battery_state.voltage = voltage_val

        percentage = (voltage_val - self.min_battery_voltage) / (self.max_battery_voltage - self.min_battery_voltage)
        percentage = max(min(percentage, 1.0), 0.0)
        battery_state.percentage = percentage

        self.battery_state_publisher.publish(battery_state)

    def command_callback(self, msg):
        try:
            # If the Roboclaw is not connected, exit the function to avoid errors when calling Roboclaw
            if not self.connect_to_roboclaw():
                return
            
            # self.get_logger().info(f"{msg}")
            # self.get_logger().info(f"{self.rclaw.ser.out_waiting}")

            # Unpack the tuple returned by twist_to_motor_commands function into two variables
            # left_motor_command and right_motor_command [-127, 127]
            left_motor_command, right_motor_command = self.twist_to_motor_commands(msg)

            # Send motor commands to Roboclaw
            if left_motor_command < 0:
                # self.rclaw.BackwardM1(self.address, abs(left_motor_command))
                self.rclaw.backward_m1(self.address, abs(left_motor_command))

            if right_motor_command < 0:
                # self.rclaw.BackwardM2(self.address, abs(right_motor_command))
                self.rclaw.backward_m2(self.address, abs(right_motor_command))

            if left_motor_command >= 0:
                # self.rclaw.ForwardM1(self.address, left_motor_command)
                self.rclaw.forward_m1(self.address, left_motor_command)

            if right_motor_command >= 0:
                # self.rclaw.ForwardM2(self.address, right_motor_command)
                self.rclaw.forward_m2(self.address, right_motor_command)

        # Even though the connection is checked in the connect_to_roboclaw function,
        # the connection can be lost while program is communicating with Roboclaw,
        # so OSError exception (usually because of Input/Output error) is caught here
        except OSError:
            self.rclaw_connected = False
            self.get_logger().error("Failed to open Roboclaw, retrying...")
        
        # All known exceptions are caught, but if some unknown exception occurs,
        # it is caught here and printed to the console
        except Exception as e:
            self.get_logger().error(f"Exception 2: {e}")

    # Convert a Twist message to motor commands and return them as a tuple (left, right)
    def twist_to_motor_commands(self, msg):
        # Get the linear and angular speed from the Twist message
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Calculate the linear speed (in m/s) of each wheel
        left_wheel_linear_speed = linear_speed - (self.wheel_track / 2.0) * angular_speed
        right_wheel_linear_speed = linear_speed + (self.wheel_track / 2.0) * angular_speed

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
                (max(left_motor_command, right_motor_command) - self.max_motor_command) * self.turn_compensation_factor

                if left_motor_command > right_motor_command:
                    right_motor_command = right_motor_command - difference

                elif right_motor_command > left_motor_command:
                    left_motor_command = left_motor_command - difference

            if left_motor_command < -self.max_motor_command or right_motor_command < -self.max_motor_command:
                difference = \
                (min(left_motor_command, right_motor_command) + self.max_motor_command) * self.turn_compensation_factor

                if left_motor_command > right_motor_command:
                    left_motor_command = left_motor_command - difference

                if right_motor_command > left_motor_command:
                    right_motor_command = right_motor_command - difference

        left_motor_command = int(left_motor_command)
        right_motor_command = int(right_motor_command)
            
        # Ensure the motor commands are within the valid range (-127 to +127)
        left_motor_command = max(min(left_motor_command, self.max_motor_command), -self.max_motor_command)
        right_motor_command = max(min(right_motor_command, self.max_motor_command), -self.max_motor_command)

        self.publish_wheel_speed(linear_speed)

        return left_motor_command, right_motor_command

    def publish_wheel_speed(self, wheel_speed_val):
        wheel_speed = Float32()
        wheel_speed.data = float(abs(wheel_speed_val) * 3.6)
        self.wheel_speed_publisher.publish(wheel_speed)
    
def main():
    rclpy.init()
    roboclaw_control_node = RoboclawControlNode()
    rclpy.spin(roboclaw_control_node)
    roboclaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()