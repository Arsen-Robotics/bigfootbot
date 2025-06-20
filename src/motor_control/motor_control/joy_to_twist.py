import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')

        # Load parameters with default values
        self.declare_parameter("enable_axis", 3)
        self.declare_parameter("reverse_button", 20)

        self.declare_parameter("camera_up_button", 12)
        self.declare_parameter("camera_down_button", 13)
        self.declare_parameter("camera_left_button", 2)
        self.declare_parameter("camera_right_button", 1)
        self.declare_parameter("camera_left_quick_view_button", 6)
        self.declare_parameter("camera_right_quick_view_button", 7)
        self.declare_parameter("camera_reset_position_button", 0)

        self.declare_parameter("buzzer_button", 1)
        self.declare_parameter("headlight_button", 6)
        self.declare_parameter("beacon_light_button", 7)
        self.declare_parameter("plow_up_button", 2)
        self.declare_parameter("plow_down_button", 3)
        self.declare_parameter("offroad_drive_mode", 9)
        self.declare_parameter("normal_drive_mode", 8)
        self.declare_parameter("trim_left", 14)
        self.declare_parameter("trim_right", 15)

        self.declare_parameter("linear_axis", 2)
        self.declare_parameter("angular_axis", 0)
        self.declare_parameter("reverse_axis", 4)

        self.declare_parameter("linear_scale", 3.067)
        self.declare_parameter("angular_scale", 9.437)
        self.declare_parameter("trim_increment", 0.05)

        self.declare_parameter("min_angular_scale", 0.4)
        self.declare_parameter("max_angular_scale", 0.7)

        # Assign parameters to variables
        self.enable_axis = self.get_parameter("enable_axis").value
        self.reverse_button = self.get_parameter("reverse_button").value

        self.camera_up_button = self.get_parameter("camera_up_button").value
        self.camera_down_button = self.get_parameter("camera_down_button").value
        self.camera_left_button = self.get_parameter("camera_left_button").value
        self.camera_right_button = self.get_parameter("camera_right_button").value
        self.camera_left_quick_view_button = self.get_parameter("camera_left_quick_view_button").value
        self.camera_right_quick_view_button = self.get_parameter("camera_right_quick_view_button").value
        
        self.camera_reset_position_button = self.get_parameter("camera_reset_position_button").value
        self.buzzer_button = self.get_parameter("buzzer_button").value
        self.headlight_button = self.get_parameter("headlight_button").value
        self.beacon_light_button = self.get_parameter("beacon_light_button").value
        self.plow_up_button = self.get_parameter("plow_up_button").value
        self.plow_down_button = self.get_parameter("plow_down_button").value
        self.offroad_drive_mode = self.get_parameter("offroad_drive_mode").value
        self.trim_left = self.get_parameter("trim_left").value
        self.trim_right = self.get_parameter("trim_right").value
        
        self.normal_drive_mode = self.get_parameter("normal_drive_mode").value
        self.linear_axis = self.get_parameter("linear_axis").value
        self.angular_axis = self.get_parameter("angular_axis").value
        self.reverse_axis = self.get_parameter("reverse_axis").value
        
        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value
        self.trim_increment = self.get_parameter("trim_increment").value

        # Flags
        self.buzzer_enabled = 0
        self.headlight_enabled = 0
        self.headlight_button_pressed = 0 
        self.beacon_light_enabled = 0
        self.beacon_light_button_pressed = 0
        self.right_quickview_enabled = 0
        self.left_quickview_enabled = 0
        self.plow_moving_up = 0
        self.plow_moving_down = 0
        self.reverse_beeper_enabled = 0
        self.drive_mode = 0 # 0 is normal drive mode, 1 is offroad drive mode
        self.trim_left_pressed = 0
        self.trim_right_pressed = 0
        self.trim_value = 0

        # Calculations for normal drive mode
        self.min_angular_scale = self.angular_scale * self.get_parameter("min_angular_scale").value
        self.max_angular_scale = self.angular_scale * self.get_parameter("max_angular_scale").value

        # Create a subscription to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.command_callback,
            10)

        # Create a publisher that will publish Twist messages to the joy_vel topic
        self.twist_publisher = self.create_publisher(
            Twist,
            'joy_vel',
            10)

        # Create a publisher that will publish String messages to the arduino_gateway topic
        self.arduino_command_publisher = self.create_publisher(
            String,
            'arduino_gateway',
            10)

    # This function is called every time a message is received on the joy topic
    # It reads joy messages and converts them to Twist messages
    # and publishes them to the cmd_vel topic
    def command_callback(self, msg):
        #msg.axes[self.enable_axis] = 1
        try:
            #if msg.axes[self.enable_axis] == 1: # Check if enable button is pressed
            twist_msg = Twist()

            # Logitech quadrant provides values between -1 and +1,
            # so it is required to scale them to the range of 0 to +1
            # because this axis is used only for forward motion
            #twist_msg.linear.x = self.linear_scale * (msg.axes[self.linear_axis] + 1) / 2
            twist_msg.linear.x = self.linear_scale * (msg.axes[self.linear_axis])

            # If quadrant's most left axis is pushed all the way down (reverse button),
            # reverse mode is activated and the most right axis is used to control the speed of the robot in reverse
            # When driving backwards, angular axis should be flipped for realism
            # if msg.buttons[self.reverse_button] == 1:
            #     twist_msg.linear.x = -self.linear_scale * (msg.axes[self.reverse_axis] + 1) / 2
            #     msg.axes[self.angular_axis] = -msg.axes[self.angular_axis]

            if self.drive_mode == 0:
                # Absolute linear speed
                abs_linear_speed = abs(twist_msg.linear.x)

                # Compute dynamic angular scale (inverse relationship with linear speed)
                dynamic_angular_scale = self.angular_scale * (1 - abs_linear_speed / self.linear_scale)

                # Ensure dynamic_angular_scale doesn't go below a minimum or above a maximum value
                dynamic_angular_scale = max(dynamic_angular_scale, self.min_angular_scale)
                dynamic_angular_scale = min(dynamic_angular_scale, self.max_angular_scale)

                # Apply the dynamic angular scale to the angular velocity
                twist_msg.angular.z = dynamic_angular_scale * msg.axes[self.angular_axis]

                # twist_msg.angular.z = self.angular_scale * (msg.axes[self.angular_axis] ** 3)
                # twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]

            elif self.drive_mode == 1:
                twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]

            # Apply trim
            if twist_msg.linear.x > 0:
                twist_msg.angular.z += self.trim_value

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

            # if msg.buttons[self.camera_left_quick_view_button] == 1 and self.left_quickview_enabled == 0:  # Check if camera left button is pressed
            #     string_msg = String()
            #     string_msg.data = "3"  # Command for camera left quick view
            #     self.arduino_command_publisher.publish(string_msg)
            #     self.left_quickview_enabled = 1
            # elif msg.buttons[self.camera_left_quick_view_button] == 0 and self.left_quickview_enabled == 1: # Check if camera left button is released
            #     string_msg = String()
            #     string_msg.data = "11"  # Command to reset camera position (pan only)
            #     self.arduino_command_publisher.publish(string_msg)
            #     self.left_quickview_enabled = 0

            # if msg.buttons[self.camera_right_quick_view_button] == 1 and self.right_quickview_enabled == 0:  # Check if camera right button is pressed
            #     string_msg = String()
            #     string_msg.data = "4"  # Command for camera right quick view
            #     self.arduino_command_publisher.publish(string_msg)
            #     self.right_quickview_enabled = 1
            # elif msg.buttons[self.camera_right_quick_view_button] == 0 and self.right_quickview_enabled == 1: # Check if camera right button is released
            #     string_msg = String()
            #     string_msg.data = "11"  # Command to reset camera position (pan only)
            #     self.arduino_command_publisher.publish(string_msg)
            #     self.right_quickview_enabled = 0

            # if msg.buttons[self.camera_left_button] == 1:
            #     string_msg = String()
            #     string_msg.data = "5" # Command for camera left
            #     self.arduino_command_publisher.publish(string_msg)

            # if msg.buttons[self.camera_right_button] == 1:
            #     string_msg = String()
            #     string_msg.data = "6" # Command for camera right
            #     self.arduino_command_publisher.publish(string_msg)

            # Buzzer
            if msg.buttons[self.buzzer_button] == 1 and self.buzzer_enabled == 0:
                string_msg = String()
                string_msg.data = "7" # Command to enable buzzer
                self.arduino_command_publisher.publish(string_msg)
                self.buzzer_enabled = 1
            elif msg.buttons[self.buzzer_button] == 0 and self.buzzer_enabled == 1:
                string_msg = String()
                string_msg.data = "8" # Command to disable buzzer
                self.arduino_command_publisher.publish(string_msg)
                self.buzzer_enabled = 0

            # Headlight
            if msg.buttons[self.headlight_button] == 1 and self.headlight_button_pressed == 0:
                self.headlight_button_pressed = 1
                if self.headlight_enabled == 0:
                    string_msg = String()
                    string_msg.data = "10"  # Command to enable light
                    self.arduino_command_publisher.publish(string_msg)
                    self.headlight_enabled = 1
                else:
                    string_msg = String()
                    string_msg.data = "9"  # Command to disable light
                    self.arduino_command_publisher.publish(string_msg)
                    self.headlight_enabled = 0

            elif msg.buttons[self.headlight_button] == 0 and self.headlight_button_pressed == 1:
                self.headlight_button_pressed = 0

            # # # Snow plow
            # # if msg.buttons[self.plow_up_button] == 1 and self.plow_moving_up == 0:
            # #     string_msg = String()
            # #     string_msg.data = "12" # Command to raise plow
            # #     self.arduino_command_publisher.publish(string_msg)
            # #     self.plow_moving_up = 1
            # # elif msg.buttons[self.plow_up_button] == 0 and self.plow_moving_up == 1:
            # #     string_msg = String()
            # #     string_msg.data = "14" # Command to stop plow
            # #     self.arduino_command_publisher.publish(string_msg)
            # #     self.plow_moving_up = 0

            # # if msg.buttons[self.plow_down_button] == 1 and self.plow_moving_down == 0:
            # #     string_msg = String()
            # #     string_msg.data = "13" # Command to lower plow
            # #     self.arduino_command_publisher.publish(string_msg)
            # #     self.plow_moving_down = 1
            # # elif msg.buttons[self.plow_down_button] == 0 and self.plow_moving_down == 1:
            # #     string_msg = String()
            # #     string_msg.data = "14" # Command to stop plow
            # #     self.arduino_command_publisher.publish(string_msg)
            # #     self.plow_moving_down = 0

            # Beacon light
            if msg.buttons[self.beacon_light_button] == 1 and self.beacon_light_button_pressed == 0:
                self.beacon_light_button_pressed = 1
                if self.beacon_light_enabled == 0:
                    string_msg = String()
                    string_msg.data = "15"  # Command to enable beacon light
                    self.arduino_command_publisher.publish(string_msg)
                    self.beacon_light_enabled = 1
                else:
                    string_msg = String()
                    string_msg.data = "16"  # Command to disable beacon light
                    self.arduino_command_publisher.publish(string_msg)
                    self.beacon_light_enabled = 0

            elif msg.buttons[self.beacon_light_button] == 0 and self.beacon_light_button_pressed == 1:
                self.beacon_light_button_pressed = 0

            # # # When reversing, start beeping for people's awareness
            # # if msg.buttons[self.reverse_button] == 1:
            # #     if self.reverse_beeper_enabled == 0:
            # #         string_msg = String()
            # #         string_msg.data = "17" # Command to enable reverse beeper
            # #         self.arduino_command_publisher.publish(string_msg)
            # #         self.reverse_beeper_enabled = 1

            # # # If reverse mode is disabled, disable the reverse beeper
            # # if msg.buttons[self.reverse_button] == 0 and self.reverse_beeper_enabled == 1:
            # #     string_msg = String()
            # #     string_msg.data = "18" # Command to disable reverse beeper
            # #     self.arduino_command_publisher.publish(string_msg)
            # #     self.reverse_beeper_enabled = 0

            # # Drive mode
            # if msg.buttons[self.normal_drive_mode] == 1:
            #     self.drive_mode = 0
            
            # if msg.buttons[self.offroad_drive_mode] == 1:
            #     self.drive_mode = 1

            # # Stop when enable button is off
            # if msg.axes[self.enable_axis] < 1:
            #     twist_msg = Twist()

            #     twist_msg.linear.x = 0.0
            #     twist_msg.angular.z = 0.0

            #     self.twist_publisher.publish(twist_msg)

            # Trim
            if msg.buttons[self.trim_left] == 1 and self.trim_left_pressed == 0:
                self.trim_value += self.trim_increment
                self.trim_left_pressed = 1
            if msg.buttons[self.trim_right] == 1 and self.trim_right_pressed == 0:
                self.trim_value -= self.trim_increment
                self.trim_right_pressed = 1
            if msg.buttons[self.trim_left] == 0 and self.trim_left_pressed == 1:
                self.trim_left_pressed = 0
            if msg.buttons[self.trim_right] == 0 and self.trim_right_pressed == 1:
                self.trim_right_pressed = 0
            
        # If an exception occurs, print the exception to the console
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main():
    rclpy.init()
    joy_to_twist_node = JoyToTwistNode()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
