# BFB Arduino Gateway

## Overview
The BFB Arduino Gateway package provides a bridge between ROS 2 and Arduino hardware on the BigFootBot. It handles serial communication with an Arduino Mega, enabling control of various hardware components.

## Features
- Automatic Arduino connection management
- Reliable serial communication
- Error handling and recovery
- Command forwarding from ROS 2 to Arduino

## Prerequisites
- ROS 2 (Humble or newer)
- Python 3.8+
- pyserial library
- Arduino Mega board

## Hardware Setup
1. Connect Arduino Mega to the computer
2. Ensure proper udev rules are installed:
   ```bash
   # /etc/udev/rules.d/99-arduino-mega.rules
   ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0010", MODE="0666", SYMLINK+="arduino-mega"
   ```

## Installation
```bash
# Clone the repository into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <repository_url>

# Build the package
cd ~/ros2_ws
colcon build --packages-select bfb_arduino_gateway
```

## Usage
1. Source your ROS 2 workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Run the node:
   ```bash
   ros2 run bfb_arduino_gateway arduino_gateway_node
   ```

3. Send commands:
   ```bash
   ros2 topic pub /arduino_gateway std_msgs/msg/String "data: '<command>'"
   ```

## Node: arduino_gateway_node

### Subscribed Topics
- `/arduino_gateway` (std_msgs/String)
  - Commands to be sent to Arduino
  - Each command should be a string followed by newline
  - Queue size: 10 messages

### Parameters
- `comport` (string, default: "/dev/arduino-mega")
  - Serial port for Arduino communication
- `baudrate` (int, default: 9600)
  - Serial communication baud rate

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

## License
[Add License Information]

## Contact
Maintainer: Jevgeni Kalbin (jevgeni.kalbin@gmail.com) 