#!/bin/bash

echo "Building WebRTC video..."
mkdir -p bin

g++ -o bin/signaling src/signaling.cpp -ljsoncpp -lpthread
if [ $? -ne 0 ]; then
    echo "Build failed for signaling."
    exit 1  # Exit if signaling build fails
fi

g++ -o bin/send src/send.cpp $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-sdp-1.0 gstreamer-webrtc-1.0) -ljsoncpp -lX11
if [ $? -ne 0 ]; then
    echo "Build failed for send."
    exit 1  # Exit if send build fails
fi

g++ -o bin/recv src/recv.cpp $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-sdp-1.0 gstreamer-webrtc-1.0) -ljsoncpp -lX11
if [ $? -ne 0 ]; then
    echo "Build failed for recv."
    exit 1  # Exit if recv build fails
fi

echo "Build successful!"