import sys
from bfb_interfaces.srv import RelayControl
import rclpy
from rclpy.node import Node
from pynput import keyboard

class RelayControlClient(Node):
    def __init__(self):
        super().__init__('relay_control_client')
        self.client = self.create_client(RelayControl, 'relay_control')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.request = RelayControl.Request()

    def send_request(self, state):
        self.request.state = state
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def on_press(self, key):
        try:
            if key.char == 'w':
                response = self.send_request("on")
                self.get_logger().info(f"{response.confirmation}")
            elif key.char == 's':
                response = self.send_request("off")
                self.get_logger().info(f"{response.confirmation}")
        except AttributeError:
            pass


def main():
    rclpy.init()

    relay_control_client = RelayControlClient()
    #response = relay_control_client.send_request("on")
    #relay_control_client.get_logger().info(f"{response.confirmation}")

    listener = keyboard.Listener(on_press=relay_control_client.on_press)
    listener.start()
    listener.join()

    relay_control_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()