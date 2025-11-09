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
import { WebRTCClient } from './webrtc-client';

/**
 * Application state and UI elements
 */
class App {
  private signalingClient: SignalingClient;
  private webrtcClient: WebRTCClient | null = null;
  private videoElement: HTMLVideoElement;
  private statusElement: HTMLElement;
  private connectButton: HTMLButtonElement;
  private disconnectButton: HTMLButtonElement;
  private signalingUrlInput: HTMLInputElement;

  constructor() {
    // Get UI elements
    // Why: Centralized DOM element access
    this.videoElement = document.getElementById('video') as HTMLVideoElement;
    this.statusElement = document.getElementById('status') as HTMLElement;
    this.connectButton = document.getElementById('connect') as HTMLButtonElement;
    this.disconnectButton = document.getElementById('disconnect') as HTMLButtonElement;
    this.signalingUrlInput = document.getElementById('signaling-url') as HTMLInputElement;

    // Initialize signaling client with default URL
    // Why: Can be changed via UI before connecting
    const defaultUrl = this.signalingUrlInput.value || 'ws://localhost:8765';
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
      const url = this.signalingUrlInput.value || 'ws://localhost:8765';
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
        onStream: (stream) => {
          // Display video stream
          // Why: Browser automatically handles H.264 decoding
          this.videoElement.srcObject = stream;
          console.log('[App] Video stream attached to video element');
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
   * Disconnect WebRTC and signaling
   */
  private disconnect(): void {
    if (this.webrtcClient) {
      this.webrtcClient.disconnect();
      this.webrtcClient = null;
    }

    // Clear video element
    // Why: Stop video playback and free resources
    this.videoElement.srcObject = null;

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
   */
  private getStatusMessage(status: string): string {
    switch (status) {
      case 'connecting':
        return 'Connecting...';
      case 'connected':
        return 'Connected - Streaming';
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

