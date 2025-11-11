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

/**
 * Track information for identifying and managing video tracks
 * 
 * Why track ID:
 * - Each camera stream needs unique identification
 * - Allows UI to map tracks to specific video elements
 * - Enables track management (start/stop individual streams)
 */
export interface TrackInfo {
  trackId: string;           // Unique identifier for this track
  track: MediaStreamTrack;   // The actual media track
  stream: MediaStream;        // Stream containing this track
  transceiver?: RTCRtpTransceiver; // WebRTC transceiver (if available)
}

export interface WebRTCClientCallbacks {
  onStatusChange?: (status: WebRTCClientStatus) => void;
  /**
   * Called when a new video track is received
   * 
   * Why per-track callback:
   * - Tracks arrive individually, not all at once
   * - Allows UI to create video elements as tracks arrive
   * - Better for dynamic stream handling
   * 
   * @param trackInfo Information about the received track
   */
  onTrack?: (trackInfo: TrackInfo) => void;
  /**
   * Legacy callback for single stream (deprecated, use onTrack instead)
   * Kept for backward compatibility but will only fire for first track
   */
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
   * Map of track IDs to track information
   * 
   * Why track all tracks:
   * - Need to manage multiple streams simultaneously
   * - Allows cleanup of specific tracks
   * - Enables track lookup by ID
   */
  private tracks: Map<string, TrackInfo> = new Map();
  
  /**
   * Counter for generating unique track IDs
   * 
   * Why sequential IDs:
   * - Simple and reliable way to identify tracks
   * - Tracks arrive in order from sender (usually)
   * - Can be enhanced later with metadata from signaling
   */
  private trackIdCounter = 0;

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

    /**
     * Handle incoming media tracks
     * 
     * Why handle each track individually:
     * - Multiple cameras send multiple tracks in same connection
     * - Each track arrives via separate ontrack event
     * - Need to process each track separately to display all cameras
     * 
     * Why track transceiver:
     * - Provides metadata about the track (mid, direction, etc.)
     * - Can be used for track identification if mid is available
     * - Useful for debugging and track management
     */
    pc.ontrack = (event) => {
      const track = event.track;
      const stream = event.streams[0]; // Get first stream (usually only one per track)
      
      console.log('[WebRTC] Received remote track:', track.kind, {
        id: track.id,
        label: track.label,
        enabled: track.enabled,
        readyState: track.readyState
      });

      // Generate unique track ID
      // Why: Need consistent ID for track management
      // Priority: Use transceiver mid if available, otherwise generate sequential ID
      const transceiver = event.transceiver;
      let trackId: string;
      
      if (transceiver && transceiver.mid) {
        // Use media section ID from SDP if available
        // Why: Matches sender's track identification
        trackId = transceiver.mid;
        console.log('[WebRTC] Using transceiver mid as track ID:', trackId);
      } else {
        // Generate sequential ID
        // Why: Fallback when mid is not available
        trackId = `track-${this.trackIdCounter++}`;
        console.log('[WebRTC] Generated track ID:', trackId);
      }

      // Create track info object
      // Why: Bundle all track-related information together
      const trackInfo: TrackInfo = {
        trackId,
        track,
        stream: stream || new MediaStream([track]), // Ensure stream exists
        transceiver
      };

      // Store track for later reference
      // Why: Need to track all active streams for cleanup and management
      this.tracks.set(trackId, trackInfo);

      // Update connection status when first track arrives
      // Why: Consider connected when we receive at least one video stream
      if (this.status !== 'connected') {
        this.setStatus('connected');
      }

      // Notify callback about new track
      // Why: UI needs to create video element for this track
      this.callbacks.onTrack?.(trackInfo);

      // Legacy callback support (only fires for first track)
      // Why: Backward compatibility, but deprecated
      if (this.tracks.size === 1 && this.callbacks.onStream) {
        console.log('[WebRTC] Firing legacy onStream callback for first track');
        this.callbacks.onStream(trackInfo.stream);
      }

      // Handle track ended event
      // Why: Track might stop (camera disconnects, etc.)
      track.onended = () => {
        console.log('[WebRTC] Track ended:', trackId);
        this.tracks.delete(trackId);
        
        // Update status if no tracks remain
        if (this.tracks.size === 0) {
          this.setStatus('disconnected');
        }
      };
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
   * - Clears track tracking map
   */
  disconnect(): void {
    if (this.peerConnection) {
      // Stop all tracks
      // Why: Release media resources and stop video decoding
      this.peerConnection.getReceivers().forEach(receiver => {
        if (receiver.track) {
          receiver.track.stop();
        }
      });

      // Also stop tracks from our tracking map
      // Why: Ensure all tracks are stopped, even if not in receivers
      this.tracks.forEach(trackInfo => {
        trackInfo.track.stop();
      });

      // Clear track tracking
      // Why: Reset state for next connection
      this.tracks.clear();
      this.trackIdCounter = 0;

      // Close peer connection
      this.peerConnection.close();
      this.peerConnection = null;
    }

    this.setStatus('disconnected');
    console.log('[WebRTC] Disconnected');
  }

  /**
   * Get all active tracks
   * 
   * Why: Allows UI to query active streams
   * - Useful for displaying stream count
   * - Enables track management UI
   */
  getTracks(): TrackInfo[] {
    return Array.from(this.tracks.values());
  }

  /**
   * Get track count
   * 
   * Why: Quick way to check how many streams are active
   */
  getTrackCount(): number {
    return this.tracks.size;
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

