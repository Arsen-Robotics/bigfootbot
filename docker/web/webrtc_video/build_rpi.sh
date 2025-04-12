#!/bin/bash

# Build script for Raspberry Pi WebRTC components

# Check if required dependencies are installed
check_dependencies() {
    echo "Checking dependencies..."
    MISSING_DEPS=0
    
    # Required packages
    PACKAGES=(
        "libgstreamer1.0-dev"
        "libgstreamer-plugins-base1.0-dev"
        "gstreamer1.0-plugins-good"
        "gstreamer1.0-plugins-bad"
        "gstreamer1.0-plugins-ugly"
        "gstreamer1.0-libav"
        "libwebsocketpp-dev"
        "libjsoncpp-dev"
        "libboost-system-dev"
        "libboost-thread-dev"
    )
    
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg -l | grep -q "$pkg"; then
            echo "Missing package: $pkg"
            MISSING_DEPS=1
        fi
    done
    
    if [ $MISSING_DEPS -eq 1 ]; then
        echo "Please install missing dependencies with:"
        echo "sudo apt update"
        echo "sudo apt install -y ${PACKAGES[*]}"
        exit 1
    fi
    
    echo "All dependencies are installed."
}

# Create bin directory if it doesn't exist
mkdir -p bin

# Check dependencies
check_dependencies

# Compiler flags
CFLAGS="-O2 -march=armv8-a -mtune=cortex-a72"
LDFLAGS="-lpthread -lwebsocketpp -ljsoncpp -lboost_system -lboost_thread"
GST_CFLAGS="$(pkg-config --cflags gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)"
GST_LDFLAGS="$(pkg-config --libs gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)"

# Build signaling server
echo "Building signaling server..."
g++ $CFLAGS src/signaling.cpp -o bin/signaling $LDFLAGS || {
    echo "Failed to build signaling server!"
    exit 1
}
echo "Signaling server build completed."

# Build WebRTC sender
echo "Building WebRTC sender for Raspberry Pi..."
g++ $CFLAGS $GST_CFLAGS src/send_rpi.cpp -o bin/send_rpi $GST_LDFLAGS $LDFLAGS || {
    echo "Failed to build WebRTC sender!"
    exit 1
}
echo "WebRTC sender build completed."

echo "Build completed successfully!"
echo "Run the signaling server: ./bin/signaling"
echo "Run the WebRTC sender: ./bin/send_rpi [server_address] [server_port]" 