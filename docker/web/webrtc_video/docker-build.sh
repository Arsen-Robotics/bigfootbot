#!/bin/bash
# WebRTC build script for Raspberry Pi (Docker environment)
# Handles compilation of signaling server and WebRTC sender

set -e  # Exit immediately if a command exits with non-zero status

# Create output directories
echo "Setting up build environment..."
mkdir -p bin

# Verify required tools are available
echo "Checking build dependencies..."
if ! command -v g++ &> /dev/null; then
    echo "ERROR: g++ compiler not found!"
    echo "Installing build-essential package..."
    apt-get update && apt-get install -y build-essential
    if ! command -v g++ &> /dev/null; then
        echo "FATAL: Failed to install g++. Build cannot continue."
        exit 1
    fi
fi

# Check for required libraries
for lib in "jsoncpp" "boost_system" "boost_thread" "gstreamer-1.0" "gstreamer-webrtc-1.0" "gstreamer-sdp-1.0"; do
    if ! ldconfig -p | grep -q "lib$lib"; then
        echo "WARNING: $lib library may not be properly installed"
    fi
done

# Compiler flags with optimization for Raspberry Pi 4
CFLAGS="-O2 -march=armv8-a -mtune=cortex-a72"
LDFLAGS="-lpthread -ljsoncpp -lboost_system -lboost_thread"

# Use pkg-config instead of hardcoded paths for GStreamer
if ! command -v pkg-config &> /dev/null; then
    echo "pkg-config not found. Installing..."
    apt-get update && apt-get install -y pkg-config
fi

# Get GStreamer flags via pkg-config
GST_INCLUDE=$(pkg-config --cflags gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
GST_LIBS=$(pkg-config --libs gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)

# Verify pkg-config found required packages
if [ $? -ne 0 ]; then
    echo "Failed to get GStreamer flags from pkg-config!"
    echo "Missing GStreamer development packages. Installing them..."
    apt-get update && apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
    
    # Try again with pkg-config
    GST_INCLUDE=$(pkg-config --cflags gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
    GST_LIBS=$(pkg-config --libs gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
    
    if [ $? -ne 0 ]; then
        echo "Still can't get GStreamer flags. Falling back to hardcoded paths."
        GST_INCLUDE="-I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include"
        GST_LIBS="-lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgstreamer-webrtc-1.0 -lgstreamer-sdp-1.0"
    fi
fi

echo "Using GStreamer Include flags: $GST_INCLUDE"
echo "Using GStreamer Library flags: $GST_LIBS"

# Build signaling server
echo "Building signaling server..."
g++ $CFLAGS src/signaling.cpp -o bin/signaling $LDFLAGS || {
    echo "ERROR: Failed to build signaling server!"
    exit 1
}
echo "✓ Signaling server built successfully"

# Build WebRTC sender
echo "Building WebRTC sender for Raspberry Pi..."
g++ $CFLAGS $GST_INCLUDE src/send_rpi.cpp -o bin/send_rpi $GST_LIBS $LDFLAGS || {
    echo "ERROR: Failed to build WebRTC sender!"
    exit 1
}
echo "✓ WebRTC sender built successfully"

echo "Build completed successfully!"
echo "Usage:"
echo "  Run the signaling server: ./bin/signaling"
echo "  Run the WebRTC sender: ./bin/send_rpi [server_address] [server_port]" 