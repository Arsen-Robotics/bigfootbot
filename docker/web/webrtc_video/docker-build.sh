#!/bin/bash
# Build script for Raspberry Pi WebRTC components (Docker-specific version)
# Uses pkg-config to find GStreamer packages

set -e  # Exit immediately if a command exits with non-zero status

echo "Creating bin directory..."
mkdir -p bin

# Verify build tools are available
if ! command -v g++ &> /dev/null; then
    echo "g++ not found. Installing build-essential..."
    apt-get update && apt-get install -y build-essential
fi

# Use pkg-config to find packages
if ! command -v pkg-config &> /dev/null; then
    echo "pkg-config not found. Installing..."
    apt-get update && apt-get install -y pkg-config
fi

# Compiler flags
CFLAGS="-O2 -march=armv8-a -mtune=cortex-a72"
LDFLAGS="-lpthread -ljsoncpp -lboost_system -lboost_thread"

# Get GStreamer flags via pkg-config - note that webrtc is part of plugins-bad, not a separate dev package
GST_INCLUDE=$(pkg-config --cflags gstreamer-1.0 gstreamer-plugins-bad-1.0 gstreamer-sdp-1.0 2>/dev/null || \
  echo "-I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include")

GST_LIBS=$(pkg-config --libs gstreamer-1.0 gstreamer-plugins-bad-1.0 gstreamer-sdp-1.0 2>/dev/null || \
  echo "-lgstreamer-1.0 -lgobject-2.0 -lglib-2.0")

echo "Using GStreamer Include flags: $GST_INCLUDE"
echo "Using GStreamer Library flags: $GST_LIBS"

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