# Architecture Overview

## System Architecture

```mermaid
graph TD
    A[Bigfootbot Camera] -->|Video Stream| B[GStreamer Pipeline]
    B -->|Encoded Video| C[WebRTC Sender]
    C -->|Signaling| D[Signaling Server]
    D -->|Signaling| E[WebRTC Receiver]
    E -->|Decoded Video| F[GStreamer Pipeline]
    F -->|Display| G[Client Screen]
    
    subgraph Robot
        A
        B
        C
    end
    
    subgraph Network
        D
    end
    
    subgraph Client
        E
        F
        G
    end
```

## Component Interaction

```mermaid
sequenceDiagram
    participant Camera
    participant Sender
    participant Signaling
    participant Receiver
    participant Display
    
    Camera->>Sender: Video Stream
    Sender->>Signaling: SDP Offer
    Signaling->>Receiver: SDP Offer
    Receiver->>Signaling: SDP Answer
    Signaling->>Sender: SDP Answer
    Sender->>Receiver: ICE Candidates
    Receiver->>Sender: ICE Candidates
    Sender->>Receiver: RTP Stream
    Receiver->>Display: Decoded Video
```

## Data Flow

```mermaid
graph LR
    A[Raw Video] -->|Capture| B[Camera]
    B -->|NV12| C[Encoder]
    C -->|H.264| D[RTP]
    D -->|Network| E[Depayloader]
    E -->|H.264| F[Decoder]
    F -->|NV12| G[Display]
```

## Network Architecture

```mermaid
graph TD
    A[Robot] -->|WebSocket| B[Signaling Server]
    B -->|WebSocket| C[Client]
    A -->|RTP/RTCP| C
    D[STUN Server] -->|ICE| A
    D -->|ICE| C
    E[TURN Server] -->|Relay| A
    E -->|Relay| C
```

## Security Architecture

```mermaid
graph TD
    A[Client] -->|WSS| B[Signaling Server]
    B -->|WSS| C[Robot]
    A -->|DTLS-SRTP| C
    D[Auth Service] -->|Token| A
    D -->|Token| C
    E[Firewall] -->|Rules| B
    E -->|Rules| C
```

## Component Details

### Camera Interface
- V4L2 driver support
- NV12 format output
- Configurable resolution and framerate
- Hardware acceleration support

### GStreamer Pipeline
```mermaid
graph LR
    A[v4l2src] -->|Raw Video| B[nvv4l2h264enc]
    B -->|H.264| C[rtph264pay]
    C -->|RTP| D[Network]
    E[Network] -->|RTP| F[rtph264depay]
    F -->|H.264| G[h264parse]
    G -->|H.264| H[nvv4l2h264dec]
    H -->|NV12| I[videoconvert]
    I -->|RGB| J[autovideosink]
```

### WebRTC Components
- Sender: Handles video encoding and streaming
- Receiver: Manages video decoding and display
- Signaling: Coordinates connection establishment
- ICE: Handles network connectivity

## Performance Considerations

### Latency Optimization
```mermaid
graph TD
    A[Input] -->|Low Latency| B[Encoding]
    B -->|Minimal Buffering| C[Network]
    C -->|Fast Decoding| D[Output]
    E[GPU Acceleration] --> B
    E --> D
```

### Resource Management
- GPU memory allocation
- CPU usage optimization
- Network bandwidth management
- Buffer size configuration

## Error Handling

```mermaid
graph TD
    A[Error Detection] --> B{Error Type}
    B -->|Connection| C[Reconnect]
    B -->|Encoding| D[Restart Pipeline]
    B -->|Decoding| E[Reset Decoder]
    C --> F[Recovery]
    D --> F
    E --> F
```

## Monitoring Architecture

```mermaid
graph TD
    A[Components] -->|Metrics| B[Monitoring Service]
    B -->|Alerts| C[Alert Manager]
    B -->|Logs| D[Log Aggregator]
    B -->|Stats| E[Dashboard]
    F[Admin] -->|Configuration| B
```

## Deployment Architecture

```mermaid
graph TD
    A[Docker] -->|Container| B[Robot]
    A -->|Container| C[Signaling Server]
    A -->|Container| D[Client]
    E[Kubernetes] -->|Orchestration| A
    F[CI/CD] -->|Deployment| E
```

## Scaling Considerations

### Horizontal Scaling
- Multiple signaling servers
- Load balancing
- Session distribution
- Resource allocation

### Vertical Scaling
- GPU resource management
- Memory optimization
- CPU core utilization
- Network bandwidth allocation

## Future Architecture

### Planned Improvements
- WebRTC data channel support
- Multiple camera streams
- Adaptive bitrate streaming
- Enhanced security features
- Cloud integration
- Edge computing support 