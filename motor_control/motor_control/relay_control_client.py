import rclpy
from rclpy.node import Node
from bfb_interfaces.srv import RelayControl
import time

class RelayControlClient(Node):
    def __init__(self):
        super().__init__('relay_control_client')
        self.client = self.create_client(RelayControl,'relay_control')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_relay_control_request(self):
        request = RelayControl.Request()
        request.state = "on"
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    relay_control_client = RelayControlClient()
    relay_control_client.send_relay_control_request()
    while rclpy.ok():
        rclpy.spin_once(relay_control_client)
        if relay_control_client.future.done():
            try:
                response = relay_control_client.future.result()
            except Exception as e:
                relay_control_client.get_logger().info(
                    f"Service call failed {e}")
            else:
                relay_control_client.get_logger().info(response.confirmation)
            break

    relay_control_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
