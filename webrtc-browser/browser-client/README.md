# WebRTC Browser Client

Browser-based WebRTC client for receiving video streams from robot cameras.

## Purpose

This client connects to a WebRTC sender (running on robot) via a signaling server and displays the video stream in the browser.

## Installation

```bash
npm install
```

## Development

```bash
# Start development server
npm run dev

# Build for production
npm run build
```

The development server will start on `http://localhost:3000`

## Usage

1. Start the signaling server (see `../signaling-server/README.md`)
2. Start the robot's WebRTC sender
3. Open `http://localhost:3000` in your browser
4. Enter the signaling server URL (default: `ws://localhost:8765`)
   - For remote robot: `ws://<robot-ip>:8765`
5. Click "Connect"
6. Video stream should appear

## Architecture

- **signaling-client.ts**: WebSocket client for signaling server communication
- **webrtc-client.ts**: WebRTC peer connection management
- **main.ts**: Application entry point and UI coordination
- **index.html**: Simple HTML interface

## Browser Compatibility

- Chrome/Edge: Full support
- Firefox: Full support
- Safari: May require additional configuration for H.264

## Troubleshooting

- **No video**: Check browser console for errors, verify signaling server is running
- **Connection fails**: Verify signaling server URL is correct and accessible
- **Black screen**: Check that robot sender is running and connected to signaling server

