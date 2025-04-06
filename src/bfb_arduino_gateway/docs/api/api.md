# API Reference

## Node: arduino_gateway_node

### Description
The main node that handles communication between ROS 2 and Arduino hardware.

### Topics

#### Subscribed Topics
- `/arduino_gateway` (std_msgs/String)
  - Commands to be sent to Arduino
  - Each command should be a string followed by newline
  - Queue size: 10 messages

### Parameters
- `comport` (string, default: "/dev/arduino-mega")
  - Serial port for Arduino communication
- `baudrate` (int, default: 9600)
  - Serial communication baud rate

### Methods

#### `__init__()`
Initializes the ArduinoGatewayNode.
- Creates subscription to `/arduino_gateway` topic
- Sets up connection timer
- Initializes connection state

#### `connect_to_arduino()`
Attempts to establish serial connection with Arduino.
- Returns: bool - Connection status
- Automatically retries on failure
- Updates connection state
- Logs connection status changes

#### `command_callback(msg)`
Handles incoming commands from ROS 2.
- Parameters:
  - msg (std_msgs/String): Command message
- Forwards commands to Arduino
- Handles communication errors
- Logs error messages

### Error Handling
The node implements comprehensive error handling:
1. Serial connection errors
2. Communication errors
3. General exceptions

### Logging
The node provides detailed logging:
- Connection status changes
- Error messages
- Command processing status

### Example Usage
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        self.publisher = self.create_publisher(String, 'arduino_gateway', 10)
        
    def send_command(self, command):
        msg = String()
        msg.data = str(command)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = ArduinoController()
    controller.send_command(1)  # Tilt camera up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Return Codes
- Success: Command forwarded to Arduino
- Failure: Error logged, command not sent
- Connection Lost: Automatic reconnection attempted

### Performance Considerations
- Queue size: 10 messages
- Timer period: 0.5 seconds
- Serial timeout: 0.2 seconds 