# Docker Setup for WebRTC Robot Camera

This guide explains how to run the WebRTC robot camera system in Docker containers on a Raspberry Pi.

## Prerequisites

1. Raspberry Pi 4 (2GB+ RAM recommended) with Raspberry Pi OS (64-bit recommended)
2. Docker and Docker Compose installed
3. USB webcam or Raspberry Pi Camera Module
4. Internet connection

## Installing Docker on Raspberry Pi

If you haven't installed Docker yet, run:

```bash
# Install Docker
curl -sSL https://get.docker.com | sh

# Add your user to the docker group (replace 'pi' with your username if different)
sudo usermod -aG docker pi

# Install Docker Compose
sudo apt-get update
sudo apt-get install -y docker-compose
```

Log out and log back in for the group changes to take effect.

## Quick Start with Docker Compose

The easiest way to run the system is with Docker Compose:

```bash
# Clone the repository (if you haven't already)
git clone https://your-repository-url.git
cd webrtc_video

# Build and start the containers
docker-compose -f docker-compose.rpi.yml up -d

# View logs
docker-compose -f docker-compose.rpi.yml logs -f
```

This will start:
- A signaling server on port 8765
- A web server for the client interface on port 8080
- The WebRTC camera sender connected to the signaling server

## Accessing the Camera Feed

1. Open a web browser on your computer (not on the Raspberry Pi)
2. Navigate to `http://<raspberry-pi-ip>:8080`
3. Click "Start Stream"
4. Enter the address of your Raspberry Pi followed by the port: `<raspberry-pi-ip>:8765`
5. The video stream should appear in your browser

## Stopping the Containers

```bash
docker-compose -f docker-compose.rpi.yml down
```

## Running Individual Containers Manually

If you prefer to run containers individually:

### 1. Build the Docker image

```bash
docker build -f Dockerfile.rpi.arm -t webrtc-rpi .
```

### 2. Run the signaling server

```bash
docker run -d --name webrtc-signaling --network=host webrtc-rpi signaling
```

### 3. Run the web client server

```bash
docker run -d --name webrtc-client --network=host webrtc-rpi web
```

### 4. Run the camera sender

```bash
docker run -d --name webrtc-sender \
  --privileged \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/vchiq:/dev/vchiq \
  --volume=/opt/vc:/opt/vc \
  --volume=/dev:/dev \
  --env="LD_LIBRARY_PATH=/opt/vc/lib" \
  --network=host \
  webrtc-rpi sender localhost 8765
```

## Camera Configuration

### Using USB Webcam

The default configuration uses a USB webcam at `/dev/video0`. If your webcam is at a different path:

1. Edit `docker-compose.rpi.yml` and change:
```yaml
devices:
  - /dev/video0:/dev/video0
```
to the correct path.

### Using Raspberry Pi Camera Module

If you're using the official Raspberry Pi Camera Module:

1. Make sure the camera module is enabled:
```bash
sudo raspi-config
# Navigate to: Interface Options > Camera > Enable
# Reboot when prompted
```

2. Edit the `send_rpi.cpp` file to use `rpicamsrc` instead of `v4l2src`:

```cpp
// Change this part of the pipeline in src/send_rpi.cpp:
v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 \

// To:
rpicamsrc ! video/x-raw,width=640,height=480,framerate=30/1 \
```

3. Rebuild the Docker image:
```bash
docker-compose -f docker-compose.rpi.yml build
```

## Troubleshooting

### Camera Not Detected

If your camera isn't detected:

```bash
# Check if the camera is visible to the host
ls -la /dev/video*

# For Raspberry Pi Camera Module
vcgencmd get_camera
```

### Container Can't Access the Camera

Make sure the container has permission to access the camera:

```bash
# Stop the containers
docker-compose -f docker-compose.rpi.yml down

# Check permissions on the device
ls -la /dev/video0

# Make it readable/writable if needed
sudo chmod 666 /dev/video0

# Start again
docker-compose -f docker-compose.rpi.yml up -d
```

### Connection Issues

If you can't connect to the signaling server:

1. Make sure there's no firewall blocking port 8765
2. Try using the IP address instead of hostname
3. Check if the signaling container is running:
```bash
docker ps | grep signaling
```

### Performance Issues

For better performance:

1. Lower the resolution and framerate in `send_rpi.cpp`
2. Add more memory to GPU:
```bash
sudo raspi-config
# Navigate to: Performance Options > GPU Memory > Set to 128MB or higher
```

## Advanced Configuration

### Running on a Different Network

If you're running the containers on a different network or behind a router:

1. Make sure ports 8765 and 8080 are forwarded to your Raspberry Pi
2. Use your public IP or domain name when connecting from the client 