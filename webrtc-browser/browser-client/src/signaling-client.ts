/**
 * WebSocket Signaling Client
 * 
 * Handles WebSocket connection to signaling server and message exchange.
 * 
 * Why separate signaling client:
 * - Separation of concerns: signaling logic separate from WebRTC logic
 * - Easier to test and maintain
 * - Can be reused if we add multiple WebRTC connections
 */

export type SignalingMessage = 
  | { status: 'HELLO' }
  | { status: 'OK' }
  | { sdp: { type: 'offer' | 'answer'; sdp: string } }
  | { ice: { candidate: string; sdpMLineIndex: number } }
  | { control?: any };

export type SignalingMessageHandler = (message: SignalingMessage) => void;

/**
 * WebSocket-based signaling client
 * 
 * Why WebSocket:
 * - Provides reliable bidirectional communication
 * - Low latency for real-time signaling
 * - Browser-native API, no additional dependencies needed
 */
export class SignalingClient {
  private ws: WebSocket | null = null;
  private url: string;
  private messageHandlers: SignalingMessageHandler[] = [];
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000; // Start with 1 second

  /**
   * @param url WebSocket server URL (e.g., 'ws://localhost:8765')
   */
  constructor(url: string) {
    this.url = url;
  }

  /**
   * Connect to signaling server
   * 
   * Why async:
   * - WebSocket connection is asynchronous
   * - Allows caller to await connection before proceeding
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        console.log(`[Signaling] Connecting to ${this.url}...`);
        this.ws = new WebSocket(this.url);

        // Why handle open immediately: Connection is ready when this fires
        this.ws.onopen = () => {
          console.log('[Signaling] Connected to server');
          this.reconnectAttempts = 0; // Reset reconnect counter on successful connection
          resolve();
        };

        // Why handle messages: Forward signaling messages to registered handlers
        this.ws.onmessage = (event) => {
          try {
            const message: SignalingMessage = JSON.parse(event.data);
            console.log('[Signaling] Received message:', this.getMessageType(message));
            this.notifyHandlers(message);
          } catch (error) {
            console.error('[Signaling] Failed to parse message:', error);
          }
        };

        // Why handle close: Attempt reconnection or notify of disconnection
        this.ws.onclose = (event) => {
          console.log(`[Signaling] Connection closed (code: ${event.code})`);
          this.ws = null;
          
          // Attempt reconnection if not a normal closure
          if (event.code !== 1000 && this.reconnectAttempts < this.maxReconnectAttempts) {
            this.attemptReconnect();
          }
        };

        // Why handle error: Log errors for debugging
        this.ws.onerror = (error) => {
          console.error('[Signaling] WebSocket error:', error);
          reject(error);
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Attempt to reconnect with exponential backoff
   * 
   * Why exponential backoff:
   * - Prevents overwhelming server with rapid reconnection attempts
   * - Gives network time to recover from temporary issues
   */
  private attemptReconnect(): void {
    this.reconnectAttempts++;
    const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);
    
    console.log(`[Signaling] Attempting reconnect ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${delay}ms...`);
    
    setTimeout(() => {
      this.connect().catch((error) => {
        console.error('[Signaling] Reconnection failed:', error);
      });
    }, delay);
  }

  /**
   * Send message to signaling server
   * 
   * Why validate connection:
   * - Prevents errors from sending on closed connection
   * - Provides clear error messages for debugging
   */
  send(message: SignalingMessage): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket is not connected');
    }

    const json = JSON.stringify(message);
    console.log('[Signaling] Sending message:', this.getMessageType(message));
    this.ws.send(json);
  }

  /**
   * Register handler for incoming messages
   * 
   * Why handler pattern:
   * - Allows multiple components to listen for signaling messages
   * - Decouples signaling from WebRTC logic
   */
  onMessage(handler: SignalingMessageHandler): void {
    this.messageHandlers.push(handler);
  }

  /**
   * Remove message handler
   */
  removeHandler(handler: SignalingMessageHandler): void {
    const index = this.messageHandlers.indexOf(handler);
    if (index > -1) {
      this.messageHandlers.splice(index, 1);
    }
  }

  /**
   * Notify all registered handlers of incoming message
   */
  private notifyHandlers(message: SignalingMessage): void {
    this.messageHandlers.forEach(handler => {
      try {
        handler(message);
      } catch (error) {
        console.error('[Signaling] Handler error:', error);
      }
    });
  }

  /**
   * Close WebSocket connection
   * 
   * Why explicit close:
   * - Clean shutdown prevents resource leaks
   * - Sends proper close code to server
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
    this.messageHandlers = [];
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  /**
   * Helper to identify message type for logging
   */
  private getMessageType(message: SignalingMessage): string {
    if ('status' in message) return `status: ${message.status}`;
    if ('sdp' in message) return `sdp: ${message.sdp.type}`;
    if ('ice' in message) return 'ice-candidate';
    if ('control' in message) return 'control';
    return 'unknown';
  }
}

