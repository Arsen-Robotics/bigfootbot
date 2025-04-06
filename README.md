# Bigfootbot (BFB)

## Overview
Bigfootbot is a versatile autonomous wheeled robot designed for multiple outdoor applications. It features a robust tank-style drive system and modular design that allows it to perform various tasks including snow cleaning, sidewalk maintenance, delivery services, and grass mowing.

## Features
- **Multi-purpose Platform**:
  - Snow cleaning with snowplow attachment
  - Sidewalk maintenance (salt/stone spreading)
  - Delivery services
  - Grass mowing capabilities

- **Advanced Hardware**:
  - Nvidia Jetson Xavier NX main computer
  - Intel Realsense D435i depth camera
  - DFRobot GPS receiver
  - RoboClaw 2x15A motor controller
  - 4G modem for remote operation
  - Two Yalu 250W DC motors with chain drive

- **Software Stack**:
  - ROS 2 Humble
  - Docker containerization
  - WebRTC for video streaming
  - Real-time teleoperation

## Package Structure
The project is organized into several ROS 2 packages:

- `bigfootbot` (metapackage)
- `bigfootbot_base` - Core robot functionality
- `bigfootbot_description` - Robot model and visualization
- `bigfootbot_teleop` - Remote control implementation
- `bigfootbot_navigation` - Autonomous navigation
- `bigfootbot_bringup` - System startup and configuration
- `bfb_arduino_gateway` - Arduino communication interface

## Installation

### Prerequisites
- ROS 2 Humble
- Docker
- Nvidia Jetpack 5.1.3 (for Jetson Xavier NX)
- Python 3.8+

### Building from Source
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone <repository_url>

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Build packages
colcon build --packages-select bigfootbot
```

### Environment Setup
Add to your `.bashrc`:
```bash
# ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Workaround for URDF and RVIZ2
export LC_NUMERIC="en_US.UTF-8"

# Gazebo models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/bigfootbot/bigfootbot_description/models
```

## Usage

### Viewing Robot Model
```bash
ros2 launch bigfootbot_description view_model.launch.py
```

### Available Launch Arguments
```bash
ros2 launch -s bigfootbot_description view_model.launch.py
```

### Example: Launch without RViz
```bash
ros2 launch bigfootbot_description view_model.launch.py use_rviz:='False'
```

## Documentation
- [Design Documentation](docs/design/design.md)
- [Usage Guide](docs/usage/usage.md)
- [API Reference](docs/api/api.md)

## Development

### Docker Support
The project includes Docker configurations for different components:
- Base image for core functionality
- Bringup configuration for system startup
- Development environment setup

### Testing
```bash
# Run tests
colcon test --packages-select bigfootbot
```

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
