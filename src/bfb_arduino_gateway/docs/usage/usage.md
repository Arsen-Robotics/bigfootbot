# Usage Guide

## Hardware Setup
1. Connect Arduino Mega to the computer
2. Ensure proper udev rules are installed:
   ```bash
   # /etc/udev/rules.d/99-arduino-mega.rules
   ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0010", MODE="0666", SYMLINK+="arduino-mega"
   ```

## Basic Usage

### Starting the Node
```bash
# Source your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Run the node
ros2 run bfb_arduino_gateway arduino_gateway_node
```

### Sending Commands
```bash
# Send a command to the Arduino
ros2 topic pub /arduino_gateway std_msgs/msg/String "data: '1'"  # Example command
```

## Command Reference

### Camera Control
- `0` - Reset camera tilt
- `1` - Tilt camera up
- `2` - Tilt camera down
- `3` - Quick look left
- `4` - Quick look right
- `5` - Pan camera left
- `6` - Pan camera right
- `11` - Reset camera pan

### Other Controls
- `7` - Buzzer on
- `8` - Buzzer off
- `9` - Light off
- `10` - Light on

## Troubleshooting

### Common Issues

1. **Arduino Not Found**
   ```bash
   # Check if Arduino device is present
   ls -l /dev/arduino-mega
   
   # Check serial port permissions
   sudo chmod 666 /dev/arduino-mega
   ```

2. **Communication Errors**
   ```bash
   # Monitor node output
   ros2 topic echo /arduino_gateway
   
   # Check node status
   ros2 node info /arduino_gateway_node
   ```

### Debug Commands
```bash
# List all topics
ros2 topic list

# Monitor node output
ros2 topic echo /arduino_gateway

# Check node status
ros2 node info /arduino_gateway_node
```

## Integration Examples

### Python Script Example
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

## Best Practices
1. Always check connection status before sending commands
2. Implement error handling in your application code
3. Use appropriate command delays for hardware response
4. Monitor node output for debugging
5. Keep Arduino firmware up to date 