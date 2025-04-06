# BFB Arduino Gateway

## Overview
The BFB Arduino Gateway package provides a bridge between ROS 2 and Arduino hardware on the BigFootBot. It handles serial communication with an Arduino Mega, enabling control of various hardware components such as servos, lights, and actuators.

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

## Documentation
- [Design Documentation](docs/design/design.md)
- [Usage Guide](docs/usage/usage.md)
- [API Reference](docs/api/api.md)

## Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License
[Add License Information]

## Contact
Maintainer: Jevgeni Kalbin (jevgeni.kalbin@gmail.com) 