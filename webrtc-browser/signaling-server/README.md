# WebRTC Signaling Server

Simple WebSocket-based signaling server for WebRTC peer connections.

## Purpose

This server facilitates WebRTC connections by forwarding signaling messages (SDP offers/answers and ICE candidates) between peers. It's compatible with the existing C++ WebRTC implementation.

## Installation

```bash
npm install
```

## Development

```bash
# Run with TypeScript (requires ts-node)
npm run dev

# Or build and run
npm run build
npm start
```

## Configuration

Environment variables:
- `PORT` - Server port (default: 8765)
- `HOST` - Server host (default: 0.0.0.0)

Example:
```bash
PORT=8765 HOST=0.0.0.0 npm run dev
```

## Protocol

The server forwards JSON messages between connected WebSocket clients. Message types:
- `{"status": "HELLO"}` - Initial handshake
- `{"status": "OK"}` - Handshake response
- `{"sdp": {"type": "offer|answer", "sdp": "..."}}` - SDP exchange
- `{"ice": {"candidate": "...", "sdpMLineIndex": 0}}` - ICE candidates

## Usage

1. Start the server
2. Connect WebSocket clients to `ws://localhost:8765`
3. Clients exchange signaling messages through the server
4. Once connected, WebRTC peers communicate directly (peer-to-peer)

