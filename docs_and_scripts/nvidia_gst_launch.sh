#!/bin/bash

# Create virtual device
sudo modprobe v4l2loopback exclusive_caps=1 video_nr=20,21 card_label="CSI Camera","Road Detection Camera"
#sudo ln -sf /dev/video20 /dev/video21

# GStreamer pipeline
gst-launch-1.0 nvarguscamerasrc sensor-mode=4 ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1' ! nvvidconv ! 'video/x-raw,format=I420' ! queue ! videoconvert ! 'video/x-raw,format=YUY2' ! v4l2sink device=/dev/video20

# docker run --rm -it --gpus all --runtime=nvidia --privileged -v ./.transitive:/root/.transitive -v /run/udev:/run/udev --name transitive_robotics_cntr transitive_robotics:latest
# sudo rmmod v4l2loopback