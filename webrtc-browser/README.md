# WebRTC Browser MVP

Minimal viable product for streaming robot camera feeds to web browsers using WebRTC.

## Overview

This project provides:
1. **Signaling Server**: Node.js/TypeScript WebSocket server for WebRTC signaling
2. **Browser Client**: TypeScript browser application for receiving video streams

Compatible with the existing C++ WebRTC sender implementation (`src/bfb_webrtc`).

## Quick Start

### 1. Start Signaling Server

```bash
cd signaling-server
npm install
npm run dev
```

Server runs on `ws://localhost:8765` by default.

### 2. Start Browser Client

```bash
cd browser-client
npm install
npm run dev
```

Open `http://localhost:3000` in your browser.

### 3. Connect Robot Sender

Start the robot's WebRTC sender (from `src/bfb_webrtc`):
```bash
ros2 run bfb_webrtc webrtc_send_node
```

### 4. Connect Browser

1. Enter signaling server URL (e.g., `ws://<robot-ip>:8765` if server is on robot)
2. Click "Connect"
3. Video stream should appear

## Architecture

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Browser   │◄───────►│   Signaling  │◄───────►│   Robot     │
│   Client    │  WebSocket  │   Server    │  WebSocket  │   Sender    │
└─────────────┘         └──────────────┘         └─────────────┘
       │                                                    │
       └──────────────── WebRTC P2P ──────────────────────┘
```

## Components

### Signaling Server (`signaling-server/`)
- WebSocket server for message forwarding
- Compatible with existing C++ implementation
- Simple, stateless design

### Browser Client (`browser-client/`)
- TypeScript/HTML5 WebRTC client
- Uses browser-native WebRTC APIs
- No plugins or additional dependencies required

## Protocol

The signaling protocol matches the existing C++ implementation:

- **HELLO/OK**: Initial handshake
- **SDP Offer/Answer**: Session description exchange
- **ICE Candidates**: NAT traversal information

## Future Enhancements

- Multiple stream support
- Stream selection UI
- Bitrate control
- Authentication/authorization
- Recording/playback
- Mobile browser support
- ERP platform integration

## Development

Each component has its own `package.json` and can be developed independently. See individual README files for details.

