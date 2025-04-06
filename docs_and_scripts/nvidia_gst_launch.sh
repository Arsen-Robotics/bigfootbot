#!/bin/bash

# Unload the V4L2 loopback module to prevent conflicts
sudo rmmod v4l2loopback

# Create virtual video devices for CSI camera and road detection overlay
sudo modprobe v4l2loopback video_nr=20,21 card_label="CSI Camera","Road Detection Overlay" exclusive_caps=1,1

# GStreamer pipeline
gst-launch-1.0 nvarguscamerasrc sensor-mode=4 ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1' ! nvvidconv ! 'video/x-raw,format=I420' ! queue ! videoconvert ! 'video/x-raw,format=YUY2' ! v4l2sink device=/dev/video20 sync=false

# docker run --rm -it --gpus all --runtime=nvidia --privileged -v ./.transitive:/root/.transitive -v /run/udev:/run/udev --name transitive_robotics_cntr transitive_robotics:latest
# sudo rmmod v4l2loopback