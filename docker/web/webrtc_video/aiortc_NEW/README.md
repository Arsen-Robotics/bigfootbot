# Robot WebRTC Signaling Server

NodeJS implementation of the WebRTC signaling server for robot camera streaming.

## Features

- WebSocket-based signaling for WebRTC connections
- Centralized ICE/TURN server configuration
- Supports both camera sender (robot) and viewer clients (browsers)
- Secure credential handling

## Running the Server

### Development

```bash
# Install dependencies
npm install

# Start the server in development mode
npm run dev
```

### Production

```bash
# Using Node directly
npm start

# Using Docker
docker build -t robot-webrtc-signaling .
docker run -p 8765:8765 robot-webrtc-signaling
```

## Environment Variables

Create a `.env` file in the root directory with the following variables:

```
PORT=8765
NODE_ENV=production
TURN_USERNAME=your_username
TURN_CREDENTIAL=your_credential
``` 