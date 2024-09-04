import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ArduinoGatewayNode(Node):
    def __init__(self):
        super().__init__('arduino_gateway_node')

        self.comport = "/dev/arduino-mega"
        self.baudrate = 9600

        # Initially these flags are set to None, because it is not known if the Arduino is connected or not
        self.arduino_connected = None

        # Create a subscription to the arduino_servos topic
        self.subscription = self.create_subscription(
            Int32,
            'arduino_servos',
            self.command_callback,
            10) # 10 is the queue size (how many messages to store in memory)

        self.timer = self.create_timer(0.5, self.connect_to_arduino)

    # This function tries to open a serial connection to the Arduino
    # If the connection succeeds or fails, it prints a message to the console
    # The flag self.arduino_connected is only for the reason
    # that the message is printed only once when the connection is established or lost
    # The function returns the value of self.arduino_connected
    def connect_to_arduino(self):
        try:
            self.serial = serial.Serial(self.comport, self.baudrate)

        except:
            if self.arduino_connected == True or self.arduino_connected == None:
                self.arduino_connected = False
                self.get_logger().error("Failed to open Arduino Mega, retrying...")

        else:
            if self.arduino_connected == False or self.arduino_connected == None:
                self.arduino_connected = True
                self.get_logger().info("Arduino Mega connected")

        return self.arduino_connected

    def command_callback(self, msg):
        try:
            # If the Arduino is not connected, exit the function to avoid errors when calling Arduino
            if not self.connect_to_arduino():
                return

            self.serial.write(f"{msg.data}\n".encode())
            
        # Even though the connection is checked in the connect_to_arduino function,
        # the connection can be lost while program is communicating with the Arduino,
        # so SerialException is caught here
        except serial.SerialException:
            self.arduino_connected = False
            self.get_logger().error("Failed to open Arduino Mega, retrying...")

        # All known exceptions are caught, but if some unknown exception occurs,
        # it is caught here and printed to the console
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main():
    rclpy.init()
    arduino_gateway_node = ArduinoGatewayNode()
    rclpy.spin(arduino_gateway_node)
    arduino_gateway_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
