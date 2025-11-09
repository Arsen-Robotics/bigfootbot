/**
 * WebRTC Signaling Server
 * 
 * This server facilitates WebRTC peer-to-peer connections by forwarding
 * signaling messages (SDP offers/answers and ICE candidates) between peers.
 * 
 * Why this approach:
 * - WebRTC requires signaling to exchange connection information before
 *   establishing a direct peer-to-peer connection
 * - WebSocket provides reliable, bidirectional communication for signaling
 * - Simple message forwarding allows any two peers to connect without
 *   complex routing logic
 * 
 * Protocol:
 * - Messages are JSON formatted
 * - Server forwards all messages from one client to all other connected clients
 * - Compatible with existing C++ sender/receiver implementation
 */

import WebSocket from 'ws';
import { createServer } from 'http';

// Configuration
const PORT = process.env.PORT ? parseInt(process.env.PORT) : 8765;
const HOST = process.env.HOST || '0.0.0.0';

/**
 * Set of all connected WebSocket clients
 * Using Set allows O(1) add/remove operations
 */
const clients = new Set<WebSocket>();

/**
 * HTTP server instance (needed for WebSocket upgrade)
 */
const server = createServer();

/**
 * WebSocket server instance
 * Handles WebSocket connections and message forwarding
 */
const wss = new WebSocket.Server({ server });

/**
 * Handle new WebSocket connections
 * 
 * Why we track connections:
 * - Need to know which clients to forward messages to
 * - Can log connection/disconnection events for debugging
 */
wss.on('connection', (ws: WebSocket) => {
  // Add client to our tracking set
  clients.add(ws);
  console.log(`[INFO] Client connected. Total connections: ${clients.size}`);

  /**
   * Handle incoming messages from this client
   * 
   * Why forward to all other clients:
   * - In a simple peer-to-peer setup, we don't know which client is sender/receiver
   * - Broadcasting ensures both peers receive the signaling messages
   * - For MVP, this is simpler than implementing room/peer matching logic
   */
  ws.on('message', (data: WebSocket.Data) => {
    try {
      // Validate that message is text (JSON)
      // Why: Binary messages could break JSON parsing downstream
      if (typeof data !== 'string') {
        console.warn('[WARN] Received non-string message, ignoring');
        return;
      }

      // Parse JSON to validate format (optional but good for debugging)
      // Why: Catch malformed messages early, log them for debugging
      try {
        const message = JSON.parse(data);
        console.log(`[DEBUG] Received message type: ${getMessageType(message)}`);
      } catch (e) {
        console.warn('[WARN] Received invalid JSON, forwarding anyway:', data.toString().substring(0, 100));
      }

      // Forward message to all other connected clients
      // Why: Simple broadcast ensures both peers get signaling messages
      let forwardedCount = 0;
      clients.forEach((client) => {
        // Don't send message back to sender
        if (client !== ws && client.readyState === WebSocket.OPEN) {
          client.send(data);
          forwardedCount++;
        }
      });

      if (forwardedCount > 0) {
        console.log(`[DEBUG] Forwarded message to ${forwardedCount} client(s)`);
      } else {
        console.log(`[DEBUG] No other clients to forward message to`);
      }
    } catch (error: unknown) {
      console.error('[ERROR] Error handling message:', error);
    }
  });

  /**
   * Handle client disconnection
   * 
   * Why cleanup is important:
   * - Prevents memory leaks from stale connections
   * - Keeps connection count accurate
   */
  ws.on('close', () => {
    clients.delete(ws);
    console.log(`[INFO] Client disconnected. Remaining connections: ${clients.size}`);
  });

  /**
   * Handle WebSocket errors
   * 
   * Why catch errors:
   * - Prevents server crashes from client-side issues
   * - Allows graceful handling of network problems
   */
  ws.on('error', (error: Error) => {
    console.error('[ERROR] WebSocket error:', error);
    // Remove client on error to prevent stale connections
    clients.delete(ws);
  });
});

/**
 * Helper function to identify message type for logging
 * 
 * Why: Better debugging - know what type of signaling message is being exchanged
 */
function getMessageType(message: any): string {
  if (message.status) return `status: ${message.status}`;
  if (message.sdp) return `sdp: ${message.sdp.type || 'unknown'}`;
  if (message.ice) return 'ice-candidate';
  if (message.control) return `control: ${message.control.action || 'unknown'}`;
  return 'unknown';
}

/**
 * Start the server
 * 
 * Why listen on 0.0.0.0:
 * - Allows connections from any network interface
 * - Important when server runs on robot and browser connects from different network
 */
server.listen(PORT, HOST, () => {
  console.log(`[INFO] WebRTC Signaling Server started`);
  console.log(`[INFO] Listening on ws://${HOST}:${PORT}`);
  console.log(`[INFO] Ready to accept WebSocket connections`);
});

/**
 * Graceful shutdown handling
 * 
 * Why: Properly close connections when server shuts down
 * - Prevents "connection reset" errors on clients
 * - Allows cleanup of resources
 */
process.on('SIGINT', () => {
  console.log('\n[INFO] Shutting down server...');
  
  // Close all client connections
  clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.close(1000, 'Server shutdown');
    }
  });
  
  // Close WebSocket server
  wss.close(() => {
    // Close HTTP server
    server.close(() => {
      console.log('[INFO] Server shut down gracefully');
      process.exit(0);
    });
  });
});

