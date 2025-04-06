#!/bin/bash

# Copy udev rules
sudo cp ~/ros2_ws/src/bigfootbot/motor_control/udev/99-roboclaw.rules /etc/udev/rules.d
sudo cp ~/ros2_ws/src/bigfootbot/bfb_gps/udev/99-gps-module.rules /etc/udev/rules.d
sudo cp ~/ros2_ws/src/bigfootbot/bfb_arduino_gateway/udev/99-arduino-mega.rules /etc/udev/rules.d
# sudo cp ~/ros2_ws/src/bigfootbot/docker/web/transitive_robotics/udev/99-usb-cameras.rules /etc/udev/rules.d

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger