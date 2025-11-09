/**
 * WebRTC Client
 * 
 * Manages WebRTC peer connection for receiving video stream from robot.
 * 
 * Why browser WebRTC API:
 * - Native browser support, no plugins needed
 * - Handles H.264 decoding automatically
 * - Low latency peer-to-peer streaming
 * - Works with existing GStreamer sender
 */

import { SignalingClient, SignalingMessage } from './signaling-client';

export type WebRTCClientStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

export interface WebRTCClientCallbacks {
  onStatusChange?: (status: WebRTCClientStatus) => void;
  onStream?: (stream: MediaStream) => void;
  onError?: (error: Error) => void;
}

/**
 * WebRTC client for receiving video stream
 * 
 * Architecture:
 * - Uses RTCPeerConnection for WebRTC connection
 * - Integrates with SignalingClient for SDP/ICE exchange
 * - Handles incoming media streams and displays them
 */
export class WebRTCClient {
  private peerConnection: RTCPeerConnection | null = null;
  private signalingClient: SignalingClient;
  private callbacks: WebRTCClientCallbacks;
  private status: WebRTCClientStatus = 'disconnected';

  /**
   * @param signalingClient Signaling client for SDP/ICE exchange
   * @param callbacks Event callbacks
   */
  constructor(signalingClient: SignalingClient, callbacks: WebRTCClientCallbacks = {}) {
    this.signalingClient = signalingClient;
    this.callbacks = callbacks;

    // Register signaling message handler
    // Why: Need to process SDP offers and ICE candidates from signaling server
    this.signalingClient.onMessage((message) => this.handleSignalingMessage(message));
  }

  /**
   * Create and configure RTCPeerConnection
   * 
   * Why configure ICE servers:
   * - STUN helps discover public IP and NAT type
   * - TURN (if needed) provides relay for strict NATs
   * - Google's STUN server is free and reliable for MVP
   */
  private createPeerConnection(): RTCPeerConnection {
    const configuration: RTCConfiguration = {
      iceServers: [
        {
          urls: 'stun:stun.l.google.com:19302'
          // Why Google STUN: Free, reliable, same as used in C++ implementation
        }
        // Note: TURN server can be added here if needed for strict NATs
      ],
      bundlePolicy: 'max-bundle' // Why: Matches C++ sender configuration
    };

    const pc = new RTCPeerConnection(configuration);

    // Handle incoming media streams
    // Why: When remote stream is added, we get the video track
    pc.ontrack = (event) => {
      console.log('[WebRTC] Received remote track:', event.track.kind);
      if (event.streams && event.streams[0]) {
        this.setStatus('connected');
        this.callbacks.onStream?.(event.streams[0]);
      }
    };

    // Handle ICE candidates
    // Why: ICE candidates need to be exchanged via signaling for NAT traversal
    pc.onicecandidate = (event) => {
      if (event.candidate) {
        console.log('[WebRTC] Generated ICE candidate');
        this.signalingClient.send({
          ice: {
            candidate: event.candidate.candidate,
            sdpMLineIndex: event.candidate.sdpMLineIndex ?? 0
          }
        });
      } else {
        console.log('[WebRTC] ICE gathering complete');
      }
    };

    // Handle connection state changes
    // Why: Monitor connection health and update status
    pc.onconnectionstatechange = () => {
      console.log('[WebRTC] Connection state:', pc.connectionState);
      if (pc.connectionState === 'connected' || pc.connectionState === 'connecting') {
        this.setStatus('connected');
      } else if (pc.connectionState === 'disconnected' || pc.connectionState === 'failed') {
        this.setStatus('disconnected');
      }
    };

    // Handle ICE connection state changes
    // Why: More granular connection state information
    pc.oniceconnectionstatechange = () => {
      console.log('[WebRTC] ICE connection state:', pc.iceConnectionState);
      if (pc.iceConnectionState === 'failed') {
        console.error('[WebRTC] ICE connection failed');
        this.callbacks.onError?.(new Error('ICE connection failed'));
      }
    };

    return pc;
  }

  /**
   * Start WebRTC connection
   * 
   * Flow:
   * 1. Create peer connection
   * 2. Send HELLO to signaling server
   * 3. Wait for SDP offer from sender
   * 4. Create SDP answer
   * 5. Exchange ICE candidates
   * 6. Establish peer-to-peer connection
   */
  async connect(): Promise<void> {
    try {
      this.setStatus('connecting');

      // Ensure signaling is connected
      if (!this.signalingClient.isConnected()) {
        await this.signalingClient.connect();
      }

      // Create peer connection
      // Why create here: Fresh connection for each connect() call
      this.peerConnection = this.createPeerConnection();

      // Send HELLO message to initiate handshake
      // Why: Matches protocol used by C++ receiver
      this.signalingClient.send({ status: 'HELLO' });

      console.log('[WebRTC] Waiting for SDP offer from sender...');
    } catch (error) {
      this.setStatus('error');
      this.callbacks.onError?.(error as Error);
      throw error;
    }
  }

  /**
   * Handle signaling messages from server
   * 
   * Why separate handler:
   * - Processes different message types (SDP, ICE, status)
   * - Keeps WebRTC logic separate from signaling
   */
  private async handleSignalingMessage(message: SignalingMessage): Promise<void> {
    if (!this.peerConnection) {
      console.warn('[WebRTC] Received message but peer connection not initialized');
      return;
    }

    // Handle SDP offer from sender
    // Why: Sender creates offer, browser creates answer
    if ('sdp' in message && message.sdp.type === 'offer') {
      try {
        console.log('[WebRTC] Received SDP offer, creating answer...');
        
        // Set remote description (offer from sender)
        await this.peerConnection.setRemoteDescription(
          new RTCSessionDescription({
            type: 'offer',
            sdp: message.sdp.sdp
          })
        );

        // Create answer
        const answer = await this.peerConnection.createAnswer();
        
        // Set local description (our answer)
        await this.peerConnection.setLocalDescription(answer);

        // Send answer to sender via signaling
        this.signalingClient.send({
          sdp: {
            type: 'answer',
            sdp: answer.sdp || ''
          }
        });

        console.log('[WebRTC] Sent SDP answer');
      } catch (error) {
        console.error('[WebRTC] Error handling SDP offer:', error);
        this.callbacks.onError?.(error as Error);
      }
    }
    // Handle ICE candidates from sender
    // Why: Need to add remote ICE candidates for NAT traversal
    else if ('ice' in message) {
      try {
        const candidate = new RTCIceCandidate({
          candidate: message.ice.candidate,
          sdpMLineIndex: message.ice.sdpMLineIndex
        });
        await this.peerConnection.addIceCandidate(candidate);
        console.log('[WebRTC] Added remote ICE candidate');
      } catch (error) {
        console.error('[WebRTC] Error adding ICE candidate:', error);
        // Don't treat this as fatal - connection might still work
      }
    }
    // Handle status messages
    // Why: Acknowledge handshake completion
    else if ('status' in message && message.status === 'OK') {
      console.log('[WebRTC] Received OK status');
    }
  }

  /**
   * Disconnect WebRTC connection
   * 
   * Why cleanup:
   * - Closes peer connection properly
   * - Stops all tracks to free resources
   * - Prevents memory leaks
   */
  disconnect(): void {
    if (this.peerConnection) {
      // Close all tracks
      this.peerConnection.getReceivers().forEach(receiver => {
        receiver.track.stop();
      });

      // Close peer connection
      this.peerConnection.close();
      this.peerConnection = null;
    }

    this.setStatus('disconnected');
    console.log('[WebRTC] Disconnected');
  }

  /**
   * Update status and notify callbacks
   */
  private setStatus(status: WebRTCClientStatus): void {
    if (this.status !== status) {
      this.status = status;
      this.callbacks.onStatusChange?.(status);
    }
  }

  /**
   * Get current connection status
   */
  getStatus(): WebRTCClientStatus {
    return this.status;
  }
}

