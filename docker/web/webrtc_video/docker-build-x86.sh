#!/bin/bash

mkdir -p bin

echo "Compiling signaling server..."
g++ -o bin/signaling src/signaling.cpp -lboost_system -lpthread -ljsoncpp || { echo "Failed to build signaling server"; exit 1; }

echo "Compiling WebRTC video sender (x86 version)..."
g++ -o bin/send_x86 src/send_x86.cpp -DGST_USE_UNSTABLE_API `pkg-config --cflags --libs gstreamer-1.0 gstreamer-plugins-bad-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0` -lX11 -lpthread -ljsoncpp -lboost_system || { echo "Failed to build WebRTC video sender"; exit 1; }

echo "Build completed successfully." 