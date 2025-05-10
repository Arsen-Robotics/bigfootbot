#!/bin/bash

# Try to get local IP address for communication
LOCAL_IP=$(hostname -I | awk '{print $1}')
if [ -z "$LOCAL_IP" ]; then
    LOCAL_IP=$(ip route get 8.8.8.8 | awk '{print $7}')
fi
if [ -z "$LOCAL_IP" ]; then
    LOCAL_IP="localhost"
fi

echo "Starting modular WebRTC video system services..."
echo "Detected IP address: $LOCAL_IP"
echo "Signaling server will be available at ws://$LOCAL_IP:8765"
echo "HTTP server will be available at http://$LOCAL_IP:8080"

# Stop and remove containers if they exist
docker stop signaling-server webrtc-sender http-server 2>/dev/null || true
docker rm signaling-server webrtc-sender http-server 2>/dev/null || true

# Start signaling server
echo "Starting signaling server..."
docker run -d \
  --name signaling-server \
  --network host \
  -e PYTHONUNBUFFERED=1 \
  -e LOG_LEVEL=INFO \
  -e HOST=0.0.0.0 \
  -p 8765:8765 \
  $(docker build -q -f signaling.Dockerfile .)

# Wait for signaling server to start
sleep 2

# Start WebRTC sender
echo "Starting WebRTC sender..."
docker run -d \
  --name webrtc-sender \
  --network host \
  -e PYTHONUNBUFFERED=1 \
  -e LOG_LEVEL=INFO \
  -e SIGNALING_URL=ws://localhost:8765 \
  $(docker build -q -f webrtc_sender.Dockerfile .)

# Start HTTP server
echo "Starting HTTP server..."
docker run -d \
  --name http-server \
  --network host \
  -e PYTHONUNBUFFERED=1 \
  -e LOG_LEVEL=INFO \
  -e HOST=0.0.0.0 \
  -e PORT=8080 \
  -p 8080:8080 \
  $(docker build -q -f http_server.Dockerfile .)

# Print connection information
echo ""
echo "All services started!"
echo "----------------------"
echo "Connection Information:"
echo "- Web Interface: http://$LOCAL_IP:8080"
echo "- Signaling WebSocket: ws://$LOCAL_IP:8765"
echo ""
echo "To connect from mobile devices, use the above URLs"
echo "Make sure your mobile device is on the same network as this machine" 