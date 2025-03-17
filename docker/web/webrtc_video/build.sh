#!/bin/bash

echo "Building WebRTC video..."
mkdir -p bin
g++ -o bin/signaling src/signaling.cpp -ljsoncpp
g++ -o bin/send src/send.cpp $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-sdp-1.0 gstreamer-webrtc-1.0) -ljsoncpp -lX11
g++ -o bin/recv src/recv.cpp $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-sdp-1.0 gstreamer-webrtc-1.0) -ljsoncpp -lX11

if [ $? -eq 0 ]; then
    echo "Build successful!"
else
    echo "Build failed!"
fi