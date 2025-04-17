#!/bin/bash

# Build script for Raspberry Pi WebRTC components (Docker-specific version)
# This version has hardcoded include and library paths to avoid pkg-config issues

echo "Creating bin directory..."
mkdir -p bin

# Compiler flags
CFLAGS="-O2 -march=armv8-a -mtune=cortex-a72"
LDFLAGS="-lpthread -ljsoncpp -lboost_system -lboost_thread"

# GStreamer flags (hardcoded for Docker environment)
GST_INCLUDE="-I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include"
GST_LIBS="-lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgstreamer-webrtc-1.0 -lgstreamer-sdp-1.0"

# Build signaling server
echo "Building signaling server..."
g++ $CFLAGS src/signaling.cpp -o bin/signaling $LDFLAGS || {
    echo "Failed to build signaling server!"
    exit 1
}
echo "Signaling server build completed."

# Build WebRTC sender
echo "Building WebRTC sender for Raspberry Pi..."
g++ $CFLAGS $GST_INCLUDE src/send_rpi.cpp -o bin/send_rpi $GST_LIBS $LDFLAGS || {
    echo "Failed to build WebRTC sender!"
    exit 1
}
echo "WebRTC sender build completed."

echo "Build completed successfully!"
echo "Run the signaling server: ./bin/signaling"
echo "Run the WebRTC sender: ./bin/send_rpi [server_address] [server_port]" 