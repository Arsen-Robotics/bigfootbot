/**
 * Main application entry point
 * 
 * Coordinates signaling client, WebRTC client, and UI.
 * 
 * Why simple structure:
 * - MVP focus: get video streaming working first
 * - Easy to understand and debug
 * - Can refactor later as features are added
 */

import { SignalingClient } from './signaling-client';
import { WebRTCClient, TrackInfo } from './webrtc-client';

/**
 * Application state and UI elements
 * 
 * Multi-stream support:
 * - Manages multiple video elements (one per camera stream)
 * - Dynamically creates/destroys video elements as tracks arrive
 * - Provides grid layout for displaying all streams
 */
class App {
  private signalingClient: SignalingClient;
  private webrtcClient: WebRTCClient | null = null;
  private statusElement: HTMLElement;
  private connectButton: HTMLButtonElement;
  private disconnectButton: HTMLButtonElement;
  private signalingUrlInput: HTMLInputElement;
  
  /**
   * Container for video elements
   * 
   * Why: Need container to hold multiple video elements
   */
  private videoContainer: HTMLElement;
  
  /**
   * Placeholder element shown when no streams
   * 
   * Why: Better UX when no video is playing
   */
  private placeholderElement: HTMLElement;
  
  /**
   * Map of track IDs to video elements
   * 
   * Why: Need to track which video element displays which track
   * - Allows updating specific video elements
   * - Enables cleanup when tracks end
   * - Supports track management (show/hide individual streams)
   */
  private videoElements: Map<string, HTMLVideoElement> = new Map();
  
  /**
   * Map of track IDs to track labels
   * 
   * Why: Store track information for display
   * - Can show camera names/labels
   * - Useful for debugging
   * - Future: Could receive from signaling server
   */
  private trackLabels: Map<string, string> = new Map();

  constructor() {
    // Get UI elements
    // Why: Centralized DOM element access
    this.videoContainer = document.getElementById('video-container') as HTMLElement;
    this.placeholderElement = document.getElementById('placeholder') as HTMLElement;
    this.statusElement = document.getElementById('status') as HTMLElement;
    this.connectButton = document.getElementById('connect') as HTMLButtonElement;
    this.disconnectButton = document.getElementById('disconnect') as HTMLButtonElement;
    this.signalingUrlInput = document.getElementById('signaling-url') as HTMLInputElement;

    // Initialize signaling client with default URL
    // Why: Can be changed via UI before connecting
    const defaultUrl = this.signalingUrlInput.value || 'ws://87.119.173.184:8765';
    this.signalingClient = new SignalingClient(defaultUrl);

    // Setup event listeners
    this.setupEventListeners();

    // Update initial status
    this.updateStatus('disconnected', 'Ready to connect');
  }

  /**
   * Setup UI event listeners
   */
  private setupEventListeners(): void {
    // Connect button
    // Why: User-initiated connection start
    this.connectButton.addEventListener('click', () => {
      this.connect();
    });

    // Disconnect button
    // Why: Allow user to stop streaming
    this.disconnectButton.addEventListener('click', () => {
      this.disconnect();
    });

    // Update signaling URL when input changes
    // Why: Allow connecting to different servers (e.g., robot IP)
    this.signalingUrlInput.addEventListener('change', () => {
      const newUrl = this.signalingUrlInput.value;
      if (newUrl && newUrl !== this.signalingClient['url']) {
        // Recreate signaling client with new URL
        this.signalingClient.disconnect();
        this.signalingClient = new SignalingClient(newUrl);
      }
    });
  }

  /**
   * Connect to signaling server and start WebRTC
   * 
   * Flow:
   * 1. Get signaling URL from input
   * 2. Create WebRTC client
   * 3. Connect signaling
   * 4. Start WebRTC connection
   */
  private async connect(): Promise<void> {
    try {
      // Update URL if changed
      const url = this.signalingUrlInput.value || 'ws://87.119.173.184:8765';
      if (url !== (this.signalingClient as any).url) {
        this.signalingClient.disconnect();
        this.signalingClient = new SignalingClient(url);
      }

      // Create WebRTC client
      // Why create new instance: Fresh state for each connection
      this.webrtcClient = new WebRTCClient(this.signalingClient, {
        onStatusChange: (status) => {
          this.updateStatus(status, this.getStatusMessage(status));
        },
        /**
         * Handle new video track
         * 
         * Why per-track callback:
         * - Multiple tracks arrive separately
         * - Need to create video element for each track
         * - Allows displaying all camera streams simultaneously
         */
        onTrack: (trackInfo) => {
          this.handleNewTrack(trackInfo);
        },
        /**
         * Legacy callback (deprecated)
         * Only fires for first track, kept for backward compatibility
         */
        onStream: (stream) => {
          console.log('[App] Legacy onStream callback (deprecated, use onTrack)');
          // This will only fire for first track, but we handle via onTrack now
        },
        onError: (error) => {
          console.error('[App] WebRTC error:', error);
          this.updateStatus('error', `Error: ${error.message}`);
        }
      });

      // Disable connect button during connection
      this.connectButton.disabled = true;
      this.disconnectButton.disabled = false;
      this.signalingUrlInput.disabled = true;

      // Start connection
      await this.webrtcClient.connect();

    } catch (error) {
      console.error('[App] Connection error:', error);
      this.updateStatus('error', `Connection failed: ${error}`);
      this.connectButton.disabled = false;
      this.disconnectButton.disabled = true;
      this.signalingUrlInput.disabled = false;
    }
  }

  /**
   * Handle new video track
   * 
   * Why separate method:
   * - Creates video element for each track
   * - Manages track-to-video mapping
   * - Updates UI layout as tracks arrive
   */
  private handleNewTrack(trackInfo: TrackInfo): void {
    const { trackId, track, stream } = trackInfo;
    
    console.log(`[App] Handling new track: ${trackId}`, {
      trackId,
      trackKind: track.kind,
      trackLabel: track.label,
      streamId: stream.id
    });

    // Generate display label for track
    // Why: User-friendly identification of camera streams
    // Future: Could receive camera name from signaling server
    const trackLabel = this.generateTrackLabel(trackId, track);
    this.trackLabels.set(trackId, trackLabel);

    // Create video element for this track
    // Why: Each track needs its own video element to display
    const videoElement = this.createVideoElement(trackId, trackLabel);
    
    // Attach stream to video element
    // Why: Browser will decode and display the video
    videoElement.srcObject = stream;
    videoElement.autoplay = true;
    videoElement.playsInline = true;
    videoElement.play(); // Force playback immediately
    
    // Store video element reference
    // Why: Need to clean up when track ends
    this.videoElements.set(trackId, videoElement);
    
    // Hide placeholder when first stream arrives
    // Why: Better UX - show video instead of placeholder
    if (this.videoElements.size === 1) {
      this.placeholderElement.style.display = 'none';
    }

    // Update status to show stream count
    // Why: User feedback about number of active streams
    const streamCount = this.videoElements.size;
    this.updateStatus('connected', `Connected - ${streamCount} stream${streamCount > 1 ? 's' : ''} active`);
  }

  /**
   * Generate display label for track
   * 
   * Why:
   * - Provides user-friendly camera identification
   * - Uses track ID or label if available
   * - Can be enhanced with metadata from signaling server
   */
  private generateTrackLabel(trackId: string, track: MediaStreamTrack): string {
    // Try to use track label if available
    if (track.label && track.label !== '') {
      return track.label;
    }
    
    // Use track ID (e.g., "track-0", "track-1")
    // Why: Consistent naming when labels aren't available
    if (trackId.startsWith('track-')) {
      const index = parseInt(trackId.replace('track-', ''));
      return `Camera ${index + 1}`;
    }
    
    // Fallback to track ID
    return trackId;
  }

  /**
   * Create video element for a track
   * 
   * Why:
   * - Each track needs its own video element
   * - Dynamically created as tracks arrive
   * - Includes label and styling
   */
  private createVideoElement(trackId: string, label: string): HTMLVideoElement {
    // Create video element
    const video = document.createElement('video');
    video.id = `video-${trackId}`;
    video.autoplay = true;
    video.playsInline = true;
    video.muted = false; // Why: Allow audio if present (though cameras usually video-only)

    // ✨ ADD THESE CRITICAL SETTINGS:
    video.setAttribute('playsinline', 'true');  // iOS compatibility
    (video as any).disablePictureInPicture = true;  // Reduce overhead
    
    // CRITICAL: Request low-latency mode
    try {
      (video as any).requestVideoFrameCallback((now: number, metadata: any) => {
        // This forces browser into low-latency decode path
        // Continuously request next frame
        if (video.srcObject) {
          (video as any).requestVideoFrameCallback(arguments.callee);
        }
      });
    } catch (e) {
      console.warn('[App] requestVideoFrameCallback not supported');
    }
    
    // Create wrapper div for video and label
    // Why: Better layout control, can add labels/controls per stream
    const wrapper = document.createElement('div');
    wrapper.className = 'video-wrapper';
    wrapper.setAttribute('data-track-id', trackId);
    
    // Create label element
    // Why: Show which camera this is
    const labelElement = document.createElement('div');
    labelElement.className = 'video-label';
    labelElement.textContent = label;
    
    // Assemble elements
    wrapper.appendChild(labelElement);
    wrapper.appendChild(video);
    
    // Add to container
    // Why: Display in grid layout
    this.videoContainer.appendChild(wrapper);
    
    console.log(`[App] Created video element for track ${trackId}: ${label}`);
    
    return video;
  }

  /**
   * Remove video element for a track
   * 
   * Why:
   * - Clean up when track ends
   * - Free DOM resources
   * - Update UI layout
   */
  private removeVideoElement(trackId: string): void {
    const videoElement = this.videoElements.get(trackId);
    if (!videoElement) {
      return;
    }

    // Stop video playback
    // Why: Free media resources
    videoElement.srcObject = null;
    
    // Find and remove wrapper element
    // Why: Remove entire video widget from DOM
    const wrapper = videoElement.closest('.video-wrapper');
    if (wrapper) {
      wrapper.remove();
    }
    
    // Remove from tracking maps
    this.videoElements.delete(trackId);
    this.trackLabels.delete(trackId);
    
    // Show placeholder if no streams remain
    // Why: Better UX when all streams disconnected
    if (this.videoElements.size === 0) {
      this.placeholderElement.style.display = 'block';
    }
    
    console.log(`[App] Removed video element for track ${trackId}`);
  }

  /**
   * Disconnect WebRTC and signaling
   */
  private disconnect(): void {
    if (this.webrtcClient) {
      this.webrtcClient.disconnect();
      this.webrtcClient = null;
    }

    // Remove all video elements
    // Why: Clean up all video resources
    this.videoElements.forEach((_, trackId) => {
      this.removeVideoElement(trackId);
    });

    // Clear tracking maps
    this.videoElements.clear();
    this.trackLabels.clear();

    // Show placeholder
    this.placeholderElement.style.display = 'block';

    // Reset UI
    this.connectButton.disabled = false;
    this.disconnectButton.disabled = true;
    this.signalingUrlInput.disabled = false;

    this.updateStatus('disconnected', 'Disconnected');
  }

  /**
   * Update status display
   */
  private updateStatus(status: string, message: string): void {
    this.statusElement.textContent = message;
    this.statusElement.className = `status status-${status}`;
    console.log(`[App] Status: ${status} - ${message}`);
  }

  /**
   * Get human-readable status message
   * 
   * Why: Provides user feedback about connection state
   */
  private getStatusMessage(status: string): string {
    switch (status) {
      case 'connecting':
        return 'Connecting...';
      case 'connected':
        // Status will be updated with stream count in handleNewTrack
        const streamCount = this.videoElements.size;
        return streamCount > 0 
          ? `Connected - ${streamCount} stream${streamCount > 1 ? 's' : ''} active`
          : 'Connected - Waiting for streams...';
      case 'error':
        return 'Connection Error';
      case 'disconnected':
      default:
        return 'Disconnected';
    }
  }
}

/**
 * Initialize app when DOM is ready
 * 
 * Why wait for DOM:
 * - Need UI elements to exist before accessing them
 * - Prevents errors from accessing undefined elements
 */
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    new App();
  });
} else {
  // DOM already ready
  new App();
}

