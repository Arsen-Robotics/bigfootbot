# Setup and Configuration Guide

## Prerequisites

### Hardware Requirements
- NVIDIA GPU with CUDA support
- Camera device (USB or network)
- Network interface with sufficient bandwidth

### Software Requirements
- Docker with NVIDIA runtime
- GStreamer 1.0
- WebSocket support
- CUDA toolkit

## Installation

### Docker Setup
```bash
# Build the Docker image
docker build -t bfb_webrtc -f Dockerfile.webrtc .

# Run the container
docker run --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -p 8080:8080 \
    -v /dev/video0:/dev/video0 \
    bfb_webrtc
```

### Manual Installation
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libnice-dev \
    libwebrtc-audio-processing-dev

# Build components
./build.sh
```

## Configuration

### Environment Variables
```bash
# Camera settings
CAMERA_DEVICE=/dev/video0
CAMERA_WIDTH=1280
CAMERA_HEIGHT=720
CAMERA_FPS=30

# WebRTC settings
WEBRTC_PORT=8080
WEBRTC_STUN_SERVER=stun:stun.l.google.com:19302
WEBRTC_TURN_SERVER=turn:turn.example.com:3478

# GPU settings
CUDA_VISIBLE_DEVICES=0
NVIDIA_VISIBLE_DEVICES=all
```

### Configuration Files

#### config.yml
```yaml
camera:
  device: /dev/video0
  format: NV12
  width: 1280
  height: 720
  fps: 30

webrtc:
  port: 8080
  stun:
    server: stun:stun.l.google.com:19302
  turn:
    server: turn:turn.example.com:3478
    username: user
    password: pass

gstreamer:
  pipeline:
    sender: "v4l2src ! nvv4l2h264enc ! rtph264pay"
    receiver: "rtph264depay ! h264parse ! nvv4l2h264dec ! videoconvert ! autovideosink"

display:
  width: 1280
  height: 720
  fullscreen: false
```

## Network Configuration

### Port Forwarding
```bash
# Forward WebRTC port
sudo iptables -A FORWARD -p tcp --dport 8080 -j ACCEPT
sudo iptables -A FORWARD -p udp --dport 8080 -j ACCEPT
```

### Firewall Rules
```bash
# Allow WebRTC traffic
sudo ufw allow 8080/tcp
sudo ufw allow 8080/udp
```

## Testing

### Camera Test
```bash
gst-launch-1.0 v4l2src ! videoconvert ! autovideosink
```

### WebRTC Test
```bash
# Start signaling server
./signaling_server

# Start video sender
./video_sender

# Start video receiver
./video_receiver
```

## Troubleshooting

### Common Issues
1. Camera not detected
   - Check device permissions
   - Verify v4l2 support
   - Test with v4l2-ctl

2. GPU acceleration issues
   - Verify NVIDIA drivers
   - Check CUDA installation
   - Test with nvidia-smi

3. Network connectivity
   - Check firewall settings
   - Verify port forwarding
   - Test with netcat

### Logging
```bash
# Enable debug logging
export GST_DEBUG=3
export WEBRTC_DEBUG=1

# View logs
tail -f /var/log/webrtc.log
```

## Security

### SSL Configuration
```bash
# Generate certificates
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365

# Configure WebSocket server
wss://example.com:8080
```

### Authentication
```yaml
security:
  ssl:
    enabled: true
    cert: /path/to/cert.pem
    key: /path/to/key.pem
  auth:
    enabled: true
    method: token
    token: secret_token
```

## Monitoring

### System Metrics
```bash
# CPU usage
top -b -n 1

# GPU usage
nvidia-smi

# Network traffic
iftop -i eth0
```

### WebRTC Stats
```javascript
// Get connection statistics
peerConnection.getStats().then(stats => {
    console.log(stats);
});
```

## Maintenance

### Updates
```bash
# Update dependencies
sudo apt-get update
sudo apt-get upgrade

# Rebuild components
./build.sh
```

### Backup
```bash
# Backup configuration
tar -czf webrtc_config_backup.tar.gz config.yml cert.pem key.pem
```

## Support

### Documentation
- [GStreamer Documentation](https://gstreamer.freedesktop.org/documentation/)
- [WebRTC Documentation](https://webrtc.org/getting-started/overview)
- [NVIDIA Documentation](https://docs.nvidia.com/cuda/)

### Contact
- Maintainer: [Your Name]
- Email: [Your Email]
- Issue Tracker: [GitHub Issues] 