# Examples

## Basic Usage

### Starting the System
```bash
# Start signaling server
./signaling_server

# Start video sender (on robot)
./video_sender

# Start video receiver (on client)
./video_receiver
```

### Docker Deployment
```bash
# Build and run all components
docker-compose -f docker-compose_webrtc.yml up
```

## Code Examples

### Signaling Server (Python)
```python
import asyncio
import websockets
import json

class SignalingServer:
    def __init__(self):
        self.clients = set()
    
    async def handle_client(self, websocket, path):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                data = json.loads(message)
                if data['type'] == 'offer':
                    await self.broadcast_except(websocket, message)
                elif data['type'] == 'answer':
                    await self.broadcast_except(websocket, message)
                elif data['type'] == 'ice':
                    await self.broadcast_except(websocket, message)
        finally:
            self.clients.remove(websocket)
    
    async def broadcast_except(self, sender, message):
        for client in self.clients:
            if client != sender:
                await client.send(message)

server = SignalingServer()
start_server = websockets.serve(server.handle_client, '0.0.0.0', 8080)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
```

### Video Sender (C++)
```cpp
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>

class VideoSender {
    GstElement* pipeline;
    GstWebRTCBin* webrtcbin;
    
    void create_pipeline() {
        pipeline = gst_parse_launch(
            "v4l2src ! nvv4l2h264enc ! rtph264pay ! webrtcbin name=webrtc",
            NULL
        );
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "webrtc");
    }
    
    void setup_webrtc() {
        g_signal_connect(webrtcbin, "on-negotiation-needed",
            G_CALLBACK(on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate",
            G_CALLBACK(on_ice_candidate), this);
    }
    
    static void on_negotiation_needed(GstElement* webrtc, gpointer user_data) {
        // Create and send offer
    }
    
    static void on_ice_candidate(GstElement* webrtc, guint mline_index,
        gchar* candidate, gpointer user_data) {
        // Send ICE candidate
    }
};
```

### Video Receiver (Python)
```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GstWebRTC

class VideoReceiver:
    def __init__(self):
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            "webrtcbin name=webrtc ! rtph264depay ! h264parse ! "
            "nvv4l2h264dec ! videoconvert ! autovideosink"
        )
        self.webrtcbin = self.pipeline.get_by_name("webrtc")
        
    def setup_webrtc(self):
        self.webrtcbin.connect("on-negotiation-needed",
            self.on_negotiation_needed)
        self.webrtcbin.connect("on-ice-candidate",
            self.on_ice_candidate)
    
    def on_negotiation_needed(self, webrtc):
        # Create and send answer
        pass
    
    def on_ice_candidate(self, webrtc, mline_index, candidate):
        # Send ICE candidate
        pass
```

## Configuration Examples

### Camera Configuration
```yaml
camera:
  device: /dev/video0
  format: NV12
  width: 1280
  height: 720
  fps: 30
  controls:
    brightness: 50
    contrast: 50
    saturation: 50
```

### WebRTC Configuration
```yaml
webrtc:
  port: 8080
  stun:
    server: stun:stun.l.google.com:19302
  turn:
    server: turn:turn.example.com:3478
    username: user
    password: pass
  ice:
    timeout: 5000
    gathering-timeout: 5000
```

### GStreamer Pipeline Configuration
```yaml
gstreamer:
  sender:
    pipeline: "v4l2src ! nvv4l2h264enc ! rtph264pay"
    bitrate: 4000000
    keyframe-interval: 30
  receiver:
    pipeline: "rtph264depay ! h264parse ! nvv4l2h264dec ! videoconvert ! autovideosink"
    latency: 100
```

## Docker Examples

### Dockerfile
```dockerfile
FROM nvidia/cuda:11.0-base

RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libnice-dev \
    libwebrtc-audio-processing-dev

COPY . /app
WORKDIR /app

CMD ["./start.sh"]
```

### docker-compose.yml
```yaml
version: '3'
services:
  signaling:
    build:
      context: .
      dockerfile: Dockerfile.signaling
    ports:
      - "8080:8080"
    networks:
      - webrtc

  sender:
    build:
      context: .
      dockerfile: Dockerfile.sender
    devices:
      - "/dev/video0:/dev/video0"
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    networks:
      - webrtc

  receiver:
    build:
      context: .
      dockerfile: Dockerfile.receiver
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    networks:
      - webrtc

networks:
  webrtc:
    driver: bridge
```

## Testing Examples

### Camera Test
```bash
# Test camera capture
gst-launch-1.0 v4l2src ! videoconvert ! autovideosink

# Test camera with specific format
gst-launch-1.0 v4l2src ! video/x-raw,format=NV12 ! videoconvert ! autovideosink
```

### WebRTC Test
```bash
# Test signaling server
curl -X POST http://localhost:8080/offer -d '{"type":"offer","sdp":"..."}'

# Test STUN server
curl -X GET http://localhost:8080/stun
```

### Performance Test
```bash
# Measure latency
gst-launch-1.0 v4l2src ! videoconvert ! timeoverlay ! autovideosink

# Measure bandwidth
gst-launch-1.0 v4l2src ! videoconvert ! video/x-raw,format=NV12 ! \
    nvv4l2h264enc ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000
```

## Monitoring Examples

### System Metrics
```bash
# Monitor GPU usage
nvidia-smi --query-gpu=utilization.gpu --format=csv -l 1

# Monitor network traffic
iftop -i eth0

# Monitor CPU usage
top -b -n 1
```

### WebRTC Stats
```javascript
// Get connection statistics
peerConnection.getStats().then(stats => {
    stats.forEach(report => {
        console.log(report);
    });
});
```

### Logging Configuration
```yaml
logging:
  level: INFO
  file: /var/log/webrtc.log
  rotation:
    size: 10MB
    count: 5
  format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
``` 