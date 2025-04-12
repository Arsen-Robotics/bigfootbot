# Robot Camera Teleoperation with WebRTC and Raspberry Pi

This project implements a real-time camera streaming system for robot teleoperation using WebRTC. It consists of three main components:
1. **Signaling Server**: Handles WebSocket connections and message passing between peers
2. **Camera Sender**: Captures video from Raspberry Pi camera and sends it via WebRTC
3. **Browser Client**: Displays the video stream in a web browser

## Hardware Requirements

* Raspberry Pi 4 (at least 2GB RAM recommended)
* USB webcam or Raspberry Pi Camera Module
* Network connection (WiFi or Ethernet)

## Software Setup

### 1. Install Dependencies

```bash
# Update package lists
sudo apt update

# Install required packages
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libwebsocketpp-dev \
    libjsoncpp-dev \
    libboost-system-dev \
    libboost-thread-dev \
    pkg-config \
    git \
    build-essential
```

### 2. Enable Camera (for Raspberry Pi Camera Module)

If you're using the Raspberry Pi Camera Module (not needed for USB webcams):

```bash
# Open raspi-config
sudo raspi-config

# Navigate to: Interface Options > Camera > Enable
# Reboot when prompted
```

### 3. Clone the Repository

```bash
git clone https://your-repository-url.git
cd webrtc_video
```

### 4. Build the Code

```bash
# Make the build script executable
chmod +x build_rpi.sh

# Run the build script
./build_rpi.sh
```

## Running the System

### 1. Start the Signaling Server

The signaling server can run on either the Raspberry Pi itself or on a separate computer. If running it on the same Pi:

```bash
./bin/signaling
```

This will start the WebSocket server on port 8765.

### 2. Start the Camera Sender

```bash
# Format: ./bin/send_rpi [server_address] [server_port]
# If signaling server is running on the same Pi:
./bin/send_rpi 127.0.0.1 8765

# If signaling server is running on a different machine:
./bin/send_rpi 192.168.1.100 8765
```

The sender will connect to the signaling server and wait for a client to connect.

### 3. Open the Web Client

1. Copy the `client/index.html` file to a web server or simply open it in a browser
2. Click the "Start Stream" button
3. Enter the signaling server address (e.g., `192.168.1.101:8765`)
4. The browser will connect to the signaling server, negotiate a WebRTC connection with the Raspberry Pi, and display the camera feed

## Configuration

### Camera Settings

The default configuration uses a 640x480 resolution at 30fps. To change these settings, modify the pipeline in `src/send_rpi.cpp` and rebuild:

```cpp
// Original pipeline
v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 \

// Example for higher resolution:
v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1 \
```

### Camera Device

The default camera device is `/dev/video0`. If your camera is on a different device:

```cpp
// Change this line in the pipeline:
v4l2src device=/dev/video0

// To use a different device:
v4l2src device=/dev/video1
```

For the Raspberry Pi Camera Module, you might need to use a different approach:

```cpp
// Replace this line:
v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 \

// With:
rpicamsrc ! video/x-raw,width=640,height=480,framerate=30/1 \
```

## Troubleshooting

### Camera Not Detected

```bash
# List available video devices
ls -l /dev/video*

# Test camera with GStreamer
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

### GStreamer Pipeline Errors

```bash
# Test the video pipeline without WebRTC
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2h264enc ! h264parse ! avdec_h264 ! autovideosink
```

### Connection Issues

1. Ensure the signaling server is running and accessible
2. Check your network firewall settings
3. Verify the correct IP addresses are being used
4. Use simple tools to test connectivity:
   ```bash
   # Test WebSocket server
   curl --include \
        --no-buffer \
        --header "Connection: Upgrade" \
        --header "Upgrade: websocket" \
        --header "Host: 127.0.0.1:8765" \
        --header "Origin: http://127.0.0.1" \
        --header "Sec-WebSocket-Key: SGVsbG8sIHdvcmxkIQ==" \
        --header "Sec-WebSocket-Version: 13" \
        http://127.0.0.1:8765/
   ```

## Performance Optimization

For better performance on Raspberry Pi:

1. **Overclocking**: If thermal conditions allow, overclocking can improve encoding performance
   ```bash
   sudo nano /boot/config.txt
   # Add:
   over_voltage=4
   arm_freq=2000
   ```

2. **Memory Split**: Allocate more memory to GPU
   ```bash
   sudo raspi-config
   # Navigate to: Performance Options > GPU Memory > Set to 128MB or higher
   ```

3. **Resolution and Framerate**: Lower values improve performance
   ```cpp
   // Modify in src/send_rpi.cpp:
   v4l2src device=/dev/video0 ! video/x-raw,width=320,height=240,framerate=15/1 \
   ```

## Adding Robot Control

To add remote control functionality:

1. Implement a WebRTC data channel in both the sender and browser client
2. Connect the Raspberry Pi's GPIO pins to motor controllers
3. Use Python libraries (e.g., RPi.GPIO, pigpio) to interface with GPIO
4. Create a simple control protocol for movement commands

See the example code in [docs/robot_control.md](docs/robot_control.md) for implementation details.

## License

[Include your license information here] 