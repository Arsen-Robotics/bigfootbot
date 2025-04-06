import rclpy
from rclpy.node import Node
from bfb_interfaces.srv import RelayControl
import serial
import time

class RelayControlServer(Node):
    def __init__(self):
        self.debug = ''

        super().__init__('relay_control_server')
        self.server = self.create_service(RelayControl, 'relay_control', self.relay_control_callback)

        # Open the serial port
        self.serial_port = serial.Serial('/dev/cr6a-arduino-mega', baudrate=9600, timeout=1)
        time.sleep(2)

    def relay_control_callback(self, request, response):
        # Assume you have a function to send the request to the serial device
        success = self.send_command_to_serial(request.state)

        if success:
            response.confirmation = f"Relay turned {request.state}"
        else:
            response.confirmation = f"{self.debug}"

        return response

    def send_command_to_serial(self, state):
        if state == "on":
            self.serial_port.write(f"r 2\r".encode())
        elif state == "off":
            self.serial_port.write(f"r 1\r".encode())
        
        time.sleep(0.1)
        char = self.serial_port.read(1).decode() # Read one character from the serial port

        if char == '1' and state == "off":
            return True
        elif char == '2' and state == "on":
            return True
        
        self.debug = f"char: {char}, state: {state}"

def main(args=None):
    rclpy.init(args=args)
    relay_control_server = RelayControlServer()
    rclpy.spin(relay_control_server)
    relay_control_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
