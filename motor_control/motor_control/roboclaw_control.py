# Official RoboClaw Python library installation:
# Download the library: http://downloads.basicmicro.com/code/roboclaw_python.zip
# Unzip the library, move roboclaw_python folder to ~/.local/lib/python3.*/site-packages

import rclpy
from rclpy.node import Node
# from roboclaw_python.roboclaw_3 import Roboclaw
#from new_roboclaw_driver.roboclaw import Roboclaw
from .roboclaw import Roboclaw
# import importlib
# importlib.reload(Roboclaw)
from geometry_msgs.msg import Twist
import math
from bfb_interfaces.msg import RoboclawState
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
import time

class RoboclawControlNode(Node):
    def __init__(self):
        super().__init__('roboclaw_control_node')

        # Constants
        self.wheel_track = 0.65 # Distance between left and right wheel in meters
        self.wheel_diameter = 0.33 # Wheel diameter in meters
        self.max_rpm = 177.5
        self.max_motor_command = 126
        self.turn_compensation_factor = 0.5

        # Serial
        self.comport = "/dev/roboclaw" # /dev/ttyAMA0 for RPi5, ttyS0 for RPi4
        self.baudrate = 57600
        self.rclaw = Roboclaw(self.comport, self.baudrate)

        # Address is set in Basicmicro Motion Studio to determine which Roboclaw to talk to,
        # if multiple Roboclaws are connected to the same serial port
        self.address = 0x80

        # Initially this flag is set to None, because it is not known if the Roboclaw is connected or not
        self.rclaw_connected = None

        # Battery
        self.max_battery_voltage = 29.4
        self.min_battery_voltage = 21
        self.battery_wh = 652.68

        # Variables for battery range calculation depending on motors setting
        self.abs_last_m1_command = None
        self.abs_last_m2_command = None
        self.last_wheel_speed_kmh = None

        # Average wattage draw of everything in the robot (without motors)
        self.avg_static_wattage_draw = 26

        # Record motor wattage draw and wheel speed throughout the trip
        self.motor_wattage_samples = []
        self.wheel_speed_samples = []

        self.max_motor_wattage_speed_samples = 100  # Maximum number of samples to keep
        self.min_motor_wattage_speed_samples = 30  # Minimum number of samples to start calculating
        
        # Other
        self.motor_overcurrent = None
        self.max_motor_current = 30 # A

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

        # Timer will call publish_roboclaw_state function every 1.1 sec
        self.timer = self.create_timer(1.1, self.publish_roboclaw_state)

    # This function tries to open a serial connection to the Roboclaw
    # If the connection succeeds or fails, it prints a message to the console
    # The flag self.rclaw_connected is only for the reason
    # that the message is printed only once when the connection is established or lost
    # The function returns the value of self.rclaw_connected
    def connect_to_roboclaw(self):
        if self.rclaw.open():
            if self.rclaw_connected == False or self.rclaw_connected == None:
                self.rclaw_connected = True
                self.get_logger().warning("Roboclaw connected")

        else:
            if self.rclaw_connected == True or self.rclaw_connected == None:
                self.rclaw_connected = False
                self.get_logger().error("Failed to open Roboclaw, retrying...1")

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

            if currents is not None:
                # current1_val = currents[1] / 100
                # current2_val = currents[2] / 100
                m1_current_val = currents[0] / 100
                m2_current_val = currents[1] / 100

                roboclaw_state.current_1 = m1_current_val
                roboclaw_state.current_2 = m2_current_val

                if m1_current_val >= self.max_motor_current or m2_current_val >= self.max_motor_current:
                    if self.motor_overcurrent == None or self.motor_overcurrent == False:
                        self.motor_overcurrent = True
                        self.get_logger().warning(f"Motor overcurrent, stopping")

                if m1_current_val <= self.max_motor_current or m2_current_val <= self.max_motor_current:
                    if self.motor_overcurrent == None or self.motor_overcurrent == True:
                        self.get_logger().warning(f"Motor current normal")
                        self.motor_overcurrent = False

            # self.get_logger().info(f"{roboclaw_state.current_1} {roboclaw_state.current_2}")

            # Main battery voltage
            # main_battery_voltage_val = self.rclaw.ReadMainBatteryVoltage(self.address)
            main_battery_voltage_val = self.rclaw.read_main_battery_voltage(self.address)

            if main_battery_voltage_val is not None:
                # roboclaw_state.main_battery_voltage = main_battery_voltage_val[1] / 10
                roboclaw_state.main_battery_voltage = main_battery_voltage_val / 10

                self.publish_battery_state(roboclaw_state.main_battery_voltage)

            # self.get_logger().info(f"{roboclaw_state.main_battery_voltage}")

            # Temperature
            # temp1_val = self.rclaw.ReadTemp(self.address)
            temp1_val = self.rclaw.read_temperature(self.address)
            # temp2_val = self.rclaw.ReadTemp2(self.address)

            if temp1_val is not None:
                # roboclaw_state.temp1 = temp1_val[1] / 10
                roboclaw_state.temp1 = temp1_val / 10
                # roboclaw_state.temp2 = temp2_val[1] / 10

            # self.get_logger().info(f"{roboclaw_state.temp1}")
            
            if currents is not None and main_battery_voltage_val is not None and self.abs_last_m1_command is not None and self.abs_last_m2_command is not None and self.last_wheel_speed_kmh is not None:
                range_km = self.calculate_battery_range(roboclaw_state.current_1, roboclaw_state.current_2, roboclaw_state.main_battery_voltage)

                if range_km is not None:
                    roboclaw_state.battery_range_km = float(range_km)

            # Publish roboclaw state
            self.roboclaw_state_publisher.publish(roboclaw_state)

        # Even though the connection is checked in the connect_to_roboclaw function,
        # the connection can be lost while program is communicating with Roboclaw,
        # so OSError exception (usually because of Input/Output error) is caught here
        except OSError:
            self.rclaw_connected = False
            self.get_logger().error("Failed to open Roboclaw, retrying...2")
        
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

    # Calculate battery range in km
    def calculate_battery_range(self, m1_current, m2_current, battery_voltage):
        range_km = None # Always return something not to get NameError
        
        # Convert motor currents to battery wattages of motors
        # Note that Roboclaw reads only motor currents
        battery_wattage_1 = (self.abs_last_m1_command / self.max_motor_command * m1_current) * battery_voltage
        battery_wattage_2 = (self.abs_last_m2_command / self.max_motor_command * m2_current) * battery_voltage

        # Only append samples if robot is moving
        if self.abs_last_m1_command > 0 or self.abs_last_m2_command > 0:
            # Maintain max length of motor_wattage_samples
            if len(self.motor_wattage_samples) >= self.max_motor_wattage_speed_samples:
                self.motor_wattage_samples.pop(0) # Remove oldest sample
            if len(self.wheel_speed_samples) >= self.max_motor_wattage_speed_samples:
                self.wheel_speed_samples.pop(0) # Remove oldest sample

            # Append wattage and wheel speed samples
            self.motor_wattage_samples.append(battery_wattage_1 + battery_wattage_2)
            self.wheel_speed_samples.append(self.last_wheel_speed_kmh)

        # Calculate average motor wattage and average wheel speed if enough samples
        # Then estimate remaining energy and range in km
        if len(self.motor_wattage_samples) >= self.min_motor_wattage_speed_samples and len(self.wheel_speed_samples) >= self.min_motor_wattage_speed_samples:
            avg_motor_wattage = sum(self.motor_wattage_samples) / len(self.motor_wattage_samples)
            avg_wheel_speed_kmh = sum(self.wheel_speed_samples) / len(self.wheel_speed_samples)

            # Add static wattage to the average motor wattage
            current_battery_energy_consumption_w = avg_motor_wattage + self.avg_static_wattage_draw

            # Estimate remaining energy
            remaining_energy_wh = self.battery_wh * ((battery_voltage - self.min_battery_voltage) / (self.max_battery_voltage - self.min_battery_voltage))
            
            # Estimate range in km
            range_km = (remaining_energy_wh / current_battery_energy_consumption_w) * avg_wheel_speed_kmh

        return range_km

    def command_callback(self, msg):
        try:
            # If the Roboclaw is not connected, exit the function to avoid errors when calling Roboclaw
            if not self.connect_to_roboclaw():
                return
            
            # self.get_logger().info(f"{msg}")
            # self.get_logger().info(f"{self.rclaw.ser.in_waiting}")

            # Unpack the tuple returned by twist_to_motor_commands function into two variables
            # left_motor_command and right_motor_command [-127, 127]
            left_motor_command, right_motor_command = self.twist_to_motor_commands(msg)

            if self.motor_overcurrent:
                left_motor_command = 0
                right_motor_command = 0

            # Update variables for battery range calculation based on motors setting
            self.abs_last_m1_command = abs(left_motor_command)
            self.abs_last_m2_command = abs(right_motor_command)

            # self.get_logger().info(f"Cmd: {left_motor_command} {right_motor_command}")

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
            self.get_logger().error("Failed to open Roboclaw, retrying...3")
        
        # All known exceptions are caught, but if some unknown exception occurs,
        # it is caught here and printed to the console
        except Exception as e:
            self.get_logger().error(f"Exception 2: {e}")

    # def run_motor_speed_increment(self):
    #     min_speed = 10
    #     max_speed = 50
    #     speed = min_speed
    #     speed_increment = 2

    #     while True:
    #         try:
    #             # Send motor commands
    #             if not self.rclaw.forward_m1(self.address, speed):
    #                 self.get_logger().error("Failed to set M1 speed")
    #             if not self.rclaw.forward_m2(self.address, speed):
    #                 self.get_logger().error("Failed to set M2 speed")

    #             self.get_logger().info(f"Setting speed to {speed}")

    #             # Change speed incrementally
    #             if speed >= max_speed:
    #                 speed_increment = -2  # Decrease speed
    #             elif speed <= min_speed:
    #                 speed_increment = 2  # Increase speed

    #             speed += speed_increment

    #             time.sleep(0.04)  # Adjust as necessary for the state check interval

    #         except Exception as e:
    #             self.get_logger().error(f"Exception in speed increment thread: {e}")

    #         finally:
    #             self.is_running = False  # Reset the running flag
    #             # Optionally stop the motors when done
    #             self.rclaw.forward_m1(self.address, 0)
    #             self.rclaw.forward_m2(self.address, 0)

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

        # Publish wheel speed in km/h to the wheel_speed topic
        self.publish_wheel_speed(abs(linear_speed) * 3.6)

        # Update variable for battery range calculation
        self.last_wheel_speed_kmh = abs(linear_speed) * 3.6

        return left_motor_command, right_motor_command

    # Function to publish wheel speed in km/h to the wheel_speed topic
    def publish_wheel_speed(self, wheel_speed_val):
        wheel_speed = Float32()
        wheel_speed.data = float(wheel_speed_val)
        self.wheel_speed_publisher.publish(wheel_speed)
    
def main():
    rclpy.init()
    roboclaw_control_node = RoboclawControlNode()
    rclpy.spin(roboclaw_control_node)
    roboclaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
