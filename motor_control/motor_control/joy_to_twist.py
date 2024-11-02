import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')

        # Buttons related to driving
        self.enable_button = 21 # 4 for ps3 controller, 21 for Logitech quadrant, if not pressed robot is stopping
        self.reverse_button = 20 # Only for Logitech quadrant, this button is used to turn on reverse mode
        
        # Servos control buttons
        self.camera_up_button = None # 12 for ps3 controller
        self.camera_down_button = None # 13 for ps3 controller
        self.camera_left_button = None # 2 for ps3 controller
        self.camera_right_button = None # 1 for ps3 controller
        self.camera_left_quick_view_button = 6 # 14 for ps3 controller, 6 for Logitech quadrant
        self.camera_right_quick_view_button = 7 # 15 for ps3 controller, 7 for Logitech quadrant
        self.camera_reset_position_button = 0 # 3 for ps3 controller, 0 for Logitech quadrant

        # Other buttons
        self.buzzer_button = 1
        self.light_off_button = 8
        self.light_on_button = 9

        # Axes
        self.linear_axis = 2 # 1 for ps3 controller, 2 for logitech yoke
        self.angular_axis = 0 # 2 for ps3 controller, 0 for logitech yoke
        self.reverse_axis = 4 # Only for Logitech quadrant, this axis is used to control the speed of the robot in reverse mode

        # Scales
        self.linear_scale = 3.067
        self.angular_scale = 9.437

        # Flags
        self.buzzer_enabled = 0
        self.light_enabled = 0
        self.right_quickview_enabled = 0
        self.left_quickview_enabled = 0

        # Create a subscription to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.command_callback,
            10)

        # Create a publisher that will publish Twist messages to the cmd_vel topic
        self.twist_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        # Create a publisher that will publish String messages to the arduino_gateway topic
        self.arduino_command_publisher = self.create_publisher(
            String,
            'arduino_gateway',
            10)

    # This function is called every time a message is received on the joy topic
    # It reads joy messages and converts them to Twist messages
    # and publishes them to the cmd_vel topic

    # One of the two functions is commented out, because one is used for the Logitech yoke and quadrant,
    # while the other is used for the PS3 controller
    def command_callback(self, msg):
        try:
            if msg.buttons[self.enable_button] == 1: # Check if enable button is pressed
                twist_msg = Twist()

                # Logitech quadrant provides values between -1 and +1,
                # so it is required to scale them to the range of 0 to +1
                # because this axis is used only for forward motion
                twist_msg.linear.x = self.linear_scale * (msg.axes[self.linear_axis] + 1) / 2

                # If quadrant's most left axis is pushed all the way down (reverse button),
                # reverse mode is activated and the most right axis is used to control the speed of the robot in reverse
                if msg.buttons[self.reverse_button] == 1:
                    twist_msg.linear.x = -self.linear_scale * (msg.axes[self.reverse_axis] + 1) / 2

                # When driving backwards, angular axis should be flipped for realism
                if twist_msg.linear.x < 0:
                    msg.axes[self.angular_axis] = -msg.axes[self.angular_axis]

                # Absolute linear speed
                abs_linear_speed = abs(twist_msg.linear.x)

                # Compute dynamic angular scale (inverse relationship with linear speed)
                dynamic_angular_scale = self.angular_scale * (1 - abs_linear_speed / self.linear_scale)

                # Ensure dynamic_angular_scale doesn't go below a minimum or above a maximum value
                min_angular_scale = self.angular_scale * 0.3  # 30% of the max scale as a minimum
                dynamic_angular_scale = max(dynamic_angular_scale, min_angular_scale)

                max_angular_scale = self.angular_scale * 0.75 # 75% of the max scale as maximum value
                dynamic_angular_scale = min(dynamic_angular_scale, max_angular_scale)

                # Apply the dynamic angular scale to the angular velocity
                twist_msg.angular.z = dynamic_angular_scale * msg.axes[self.angular_axis]

                # twist_msg.angular.z = self.angular_scale * (msg.axes[self.angular_axis] ** 3)
                # twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]

                self.twist_publisher.publish(twist_msg)

            # if msg.buttons[self.camera_reset_position_button] == 1: # Check if camera reset position button is pressed
            #     string_msg = String()
            #     string_msg.data = "0" # Command for camera tilt reset position
            #     self.arduino_command_publisher.publish(string_msg)
            #     string_msg.data = "11" # Command for camera pan reset position
            #     self.arduino_command_publisher.publish(string_msg)

            # if msg.buttons[self.camera_up_button] == 1: # Check if camera up button is pressed
            #     string_msg = String()
            #     string_msg.data = "1" # Command for camera up
            #     self.arduino_command_publisher.publish(string_msg)

            # if msg.buttons[self.camera_down_button] == 1:  # Check if camera down button is pressed
            #     string_msg = String()
            #     string_msg.data = "2"  # Command for camera down
            #     self.arduino_command_publisher.publish(string_msg)

            if msg.buttons[self.camera_left_quick_view_button] == 1 and self.left_quickview_enabled == 0:  # Check if camera left button is pressed
                string_msg = String()
                string_msg.data = "3"  # Command for camera left quick view
                self.arduino_command_publisher.publish(string_msg)
                self.left_quickview_enabled = 1
            elif msg.buttons[self.camera_left_quick_view_button] == 0 and self.left_quickview_enabled == 1: # Check if camera left button is released
                string_msg = String()
                string_msg.data = "11"  # Command to reset camera position (pan only)
                self.arduino_command_publisher.publish(string_msg)
                self.left_quickview_enabled = 0

            if msg.buttons[self.camera_right_quick_view_button] == 1 and self.right_quickview_enabled == 0:  # Check if camera right button is pressed
                string_msg = String()
                string_msg.data = "4"  # Command for camera right quick view
                self.arduino_command_publisher.publish(string_msg)
                self.right_quickview_enabled = 1
            elif msg.buttons[self.camera_right_quick_view_button] == 0 and self.right_quickview_enabled == 1: # Check if camera right button is released
                string_msg = String()
                string_msg.data = "11"  # Command to reset camera position (pan only)
                self.arduino_command_publisher.publish(string_msg)
                self.right_quickview_enabled = 0

            # if msg.buttons[self.camera_left_button] == 1:
            #     string_msg = String()
            #     string_msg.data = "5" # Command for camera left
            #     self.arduino_command_publisher.publish(string_msg)

            # if msg.buttons[self.camera_right_button] == 1:
            #     string_msg = String()
            #     string_msg.data = "6" # Command for camera right
            #     self.arduino_command_publisher.publish(string_msg)

            if msg.buttons[self.buzzer_button] == 1 and self.buzzer_enabled == 0:
                string_msg = String()
                string_msg.data = "7" # Command to enable buzzer
                self.arduino_command_publisher.publish(string_msg)
            elif msg.buttons[self.buzzer_button] == 0 and self.buzzer_enabled == 1:
                string_msg = String()
                string_msg.data = "8" # Command to disable buzzer
                self.arduino_command_publisher.publish(string_msg)
            self.buzzer_enabled = msg.buttons[self.buzzer_button]

            if msg.buttons[self.light_off_button] == 1 and self.light_enabled == 1:
                string_msg = String()
                string_msg.data = "9" # Command to disable light
                self.arduino_command_publisher.publish(string_msg)
                self.light_enabled = 0
            elif msg.buttons[self.light_on_button] == 1 and self.light_enabled == 0:
                string_msg = String()
                string_msg.data = "10" # Command to enable light
                self.arduino_command_publisher.publish(string_msg)
                self.light_enabled = 1

        # If an exception occurs, print the exception to the console
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

    # def command_callback(self, msg):
    #     try:
    #         # if msg.buttons[self.enable_button] == 1: # Check if enable button is pressed
    #         twist_msg = Twist()

    #         # When driving backwards, angular axis should be flipped for realism
    #         if msg.axes[self.linear_axis] < 0:
    #             msg.axes[self.angular_axis] = -msg.axes[self.angular_axis]

    #         # Logitech quadrant provides values between -1 and +1,
    #         # so it is required to scale them to the range of 0 to +1
    #         # because this axis is used only for forward motion
    #         twist_msg.linear.x = self.linear_scale * msg.axes[self.linear_axis]

    #         # Absolute linear speed
    #         abs_linear_speed = abs(twist_msg.linear.x)

    #         # Compute dynamic angular scale (inverse relationship with linear speed)
    #         dynamic_angular_scale = self.angular_scale * (1 - abs_linear_speed / self.linear_scale)

    #         # Ensure dynamic_angular_scale doesn't go below a minimum or above a maximum value
    #         min_angular_scale = self.angular_scale * 0.3  # 30% of the max scale as a minimum
    #         dynamic_angular_scale = max(dynamic_angular_scale, min_angular_scale)

    #         max_angular_scale = self.angular_scale * 0.75 # 75% of the max scale as maximum value
    #         dynamic_angular_scale = min(dynamic_angular_scale, max_angular_scale)

    #         # Apply the dynamic angular scale to the angular velocity
    #         twist_msg.angular.z = dynamic_angular_scale * msg.axes[self.angular_axis]

    #         # twist_msg.angular.z = self.angular_scale * (msg.axes[self.angular_axis] ** 3)
    #         # twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]

    #         self.twist_publisher.publish(twist_msg)

    #         if msg.buttons[self.camera_reset_position_button] == 1: # Check if camera up button is pressed
    #             string_msg = String()
    #             string_msg.data = "0" # Command for camera reset position
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_up_button] == 1: # Check if camera up button is pressed
    #             string_msg = String()
    #             string_msg.data = "1" # Command for camera up
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_down_button] == 1:  # Check if camera down button is pressed
    #             string_msg = String()
    #             string_msg.data = "2"  # Command for camera down
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_left_quick_view_button] == 1:  # Check if camera left button is pressed
    #             string_msg = String()
    #             string_msg.data = "3"  # Command for camera left quick view
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_right_quick_view_button] == 1:  # Check if camera right button is pressed
    #             string_msg = String()
    #             string_msg.data = "4"  # Command for camera right quick view
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_left_button] == 1:
    #             string_msg = String()
    #             string_msg.data = "5" # Command for camera left
    #             self.arduino_command_publisher.publish(string_msg)

    #         if msg.buttons[self.camera_right_button] == 1:
    #             string_msg = String()
    #             string_msg.data = "6" # Command for camera right
    #             self.arduino_command_publisher.publish(string_msg)

    #     # If an exception occurs, print the exception to the console
    #     except Exception as e:
    #         self.get_logger().error(f"Exception: {e}")

def main():
    rclpy.init()
    joy_to_twist_node = JoyToTwistNode()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
