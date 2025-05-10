# WebRTC Video Streaming System

This project is a modular WebRTC video streaming system with three separate services:

1. **HTTP Server** - Serves the web interface
2. **WebRTC Sender** - Handles video streaming 
3. **Signaling Server** - Manages WebSocket connections for signaling

## Architecture

The system is divided into three separate services to improve maintainability and separation of concerns:

### HTTP Server (Port 8080)
- Serves the web interface (HTML, CSS, JavaScript)
- No WebRTC functionality, purely for serving static content
- Built with aiohttp

### WebRTC Sender
- Connects to the signaling server as a media server
- Handles WebRTC peer connections
- Generates video content (color bars test pattern)
- Processes offers and creates answers

### Signaling Server (Port 8765)
- WebSocket server for real-time communication
- Handles client registration
- Routes offers, answers, and ICE candidates between clients and WebRTC senders
- Manages connection state

## Installation and Setup

### Prerequisites
- Docker

### Running the System
To start all three services:

```bash
./start.sh
```

This will:
1. Build and start the signaling server on port 8765
2. Build and start the WebRTC sender
3. Build and start the HTTP server on port 8080

You can then access the web interface at http://localhost:8080

## Implementation Details

### Signaling Flow
1. Client connects to the signaling server via WebSocket
2. Client registers with the signaling server
3. WebRTC sender connects to the signaling server and registers as a media server
4. Client creates and sends an offer via the signaling server
5. Signaling server forwards the offer to the WebRTC sender
6. WebRTC sender creates an answer and sends it back via the signaling server
7. Client and WebRTC sender exchange ICE candidates via the signaling server
8. WebRTC connection is established and video streaming begins

### Environment Variables

#### HTTP Server
- `PORT`: HTTP server port (default: 8080)
- `LOG_LEVEL`: Logging level (default: INFO)

#### WebRTC Sender
- `SIGNALING_URL`: WebSocket URL for the signaling server (default: ws://localhost:8765)
- `LOG_LEVEL`: Logging level (default: INFO)

#### Signaling Server
- `LOG_LEVEL`: Logging level (default: INFO)

## Features

- Low-latency video streaming from robot cameras to browser
- Simple, all-in-one server that handles both web serving and WebRTC
- Auto-reconnect capability if connection is lost
- Support for multiple cameras (configurable)
- Containerized deployment with Docker

## Requirements

- Docker and Docker Compose
- A camera connected to the robot (accessible as a video device)

## Quick Start

1. Clone this repository
2. Navigate to the project directory:
   ```
   cd docker/web/webrtc_video/aiortc_impl
   ```
3. Start the server:
   ```
   docker-compose up --build
   ```
4. Open a web browser and navigate to:
   ```
   http://<robot-ip-address>:8080
   ```

## Configuration

### Multiple Cameras

To stream from multiple cameras, modify the `combined_server.py` file:

```python
# Add camera video track
pc.addTrack(CameraVideoTrack(camera_id=0))  # Main camera
pc.addTrack(CameraVideoTrack(camera_id=1))  # Second camera
```

Also update the `docker-compose.yml` file to include additional camera devices:

```yaml
devices:
  - "/dev/video0:/dev/video0"  # Main camera
  - "/dev/video1:/dev/video1"  # Second camera
```

### Camera Resolution

To change the camera resolution, modify the `CameraVideoTrack` class in `combined_server.py`:

```python
# Set camera properties
self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)   # Change width
self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)   # Change height
self.camera.set(cv2.CAP_PROP_FPS, 30)             # Change FPS
```

## Troubleshooting

- **No video appears**: Make sure the camera is properly connected and accessible as `/dev/video0` or the configured device.
- **High latency**: Try reducing the resolution or frame rate in the `CameraVideoTrack` class.
- **Connection issues**: Ensure the firewall allows access to port 8080.

## Implementation Details

This implementation uses:

- **aiortc**: Python WebRTC implementation
- **aiohttp**: Async web server
- **OpenCV**: Camera capture and image processing
- **Docker**: Containerization and deployment 