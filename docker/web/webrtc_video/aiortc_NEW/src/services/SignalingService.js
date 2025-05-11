const logger = require('../utils/logger');
const { iceServers } = require('../config');

/**
 * SignalingService handles all WebRTC signaling between clients and media servers
 * Uses Socket.IO for easier connection management
 */
class SignalingService {
  constructor(io) {
    // Store the Socket.IO server
    this.io = io;
    
    // Store active connections
    this.clients = new Map(); // Map of socket IDs to client sockets
    this.mediaServers = new Set(); // Set of media server socket IDs
    
    logger.info(`SignalingService initialized with ${iceServers.length} ICE servers`);
    
    // Set up Socket.IO event handlers
    this._setupSocketHandlers();
  }
  
  /**
   * Set up Socket.IO event handlers
   * @private
   */
  _setupSocketHandlers() {
    this.io.on('connection', (socket) => {
      const clientIp = socket.handshake.address;
      
      logger.info(`New connection from ${clientIp} with socket ID: ${socket.id}`);
      
      // Set default properties
      socket.isMediaServer = false;
      
      // Handle registration as media server
      socket.on('register_media_server', () => {
        this._handleMediaServerRegistration(socket);
      });
      
      // Handle registration as client
      socket.on('register', (data) => {
        this._handleClientRegistration(socket, data);
      });
      
      // Handle SDP offer
      socket.on('offer', (data) => {
        this._handleOffer(socket, data);
      });
      
      // Handle SDP answer
      socket.on('answer', (data) => {
        this._handleAnswer(socket, data);
      });
      
      // Handle ICE candidate
      socket.on('ice', (data) => {
        this._handleIceCandidate(socket, data);
      });
      
      // Handle disconnection
      socket.on('disconnect', () => {
        this._handleDisconnection(socket);
      });
      
      // Handle errors
      socket.on('error', (error) => {
        logger.error(`Socket error for ${socket.id}: ${error.message}`);
      });
    });
  }

  /**
   * Handle media server registration
   * @param {Socket} socket - The Socket.IO socket
   * @private
   */
  _handleMediaServerRegistration(socket) {
    socket.isMediaServer = true;
    this.mediaServers.add(socket.id);
    this.clients.set(socket.id, socket);
    
    logger.info(`Registered socket ${socket.id} as media server`);
    
    // Send confirmation to the media server
    socket.emit('registered_media_server', {
      id: socket.id
    });
  }

  /**
   * Handle client registration
   * @param {Socket} socket - The Socket.IO socket
   * @param {Object} data - The message data
   * @private
   */
  _handleClientRegistration(socket, data) {
    // Add to clients map
    this.clients.set(socket.id, socket);
    
    logger.info(`Client ${socket.id} registered`);

    // Send ICE servers configuration
    socket.emit('config', { 
      iceServers 
    });
    
    // Send confirmation
    socket.emit('registered', {
      id: socket.id
    });
  }

  /**
   * Handle SDP offer from client
   * @param {Socket} socket - The Socket.IO socket
   * @param {Object} data - The message data containing the offer
   * @private
   */
  _handleOffer(socket, data) {
    // Check if we have media servers available
    if (this.mediaServers.size === 0) {
      logger.warn(`No media servers available to handle offer from ${socket.id}`);
      
      // Send an error response to the client
      socket.emit('error', {
        message: 'No media servers available'
      });
      return;
    }
    
    // For now, use the first media server (load balancing could be added)
    const mediaServerId = [...this.mediaServers][0];
    const mediaServer = this.clients.get(mediaServerId);

    if (!mediaServer) {
      logger.error(`Media server ${mediaServerId} not found in clients map`);
      return;
    }

    // Extract SDP and video source preference
    const sdp = data.sdp;
    const videoSource = data.video_source || 'camera';
    
    if (!sdp) {
      logger.warn(`Received offer without SDP from ${socket.id}`);
      return;
    }
    
    // Log first line of SDP for debugging
    const firstSdpLine = sdp.split('\n')[0] || sdp.substring(0, 50);
    logger.info(`Offer SDP first line: ${firstSdpLine}`);
    
    // Forward the offer to the media server
    mediaServer.emit('offer', {
      client_id: socket.id,
      sdp,
      video_source: videoSource
    });
    
    logger.info(`Forwarded offer from ${socket.id} to media server ${mediaServerId}`);
  }

  /**
   * Handle SDP answer (from media server to client)
   * @param {Socket} socket - The Socket.IO socket
   * @param {Object} data - The message data containing the answer
   * @private
   */
  _handleAnswer(socket, data) {
    if (!socket.isMediaServer) {
      logger.warn(`Received answer from non-media server ${socket.id}`);
      return;
    }
    
    const targetClientId = data.client_id;
    const sdp = data.sdp;
    
    if (!targetClientId || !sdp) {
      logger.warn(`Invalid answer from media server: missing client_id or sdp`);
      return;
    }
    
    // Find the target client
    const clientSocket = this.clients.get(targetClientId);
    if (!clientSocket) {
      logger.warn(`Cannot forward answer: client ${targetClientId} not found`);
      return;
    }
    
    // Forward the answer to the client
    clientSocket.emit('answer', {
      sdp
    });
    
    logger.info(`Forwarded answer from media server to client ${targetClientId}`);
  }

  /**
   * Handle ICE candidate exchange
   * @param {Socket} socket - The Socket.IO socket
   * @param {Object} data - The message data containing the candidate
   * @private
   */
  _handleIceCandidate(socket, data) {
    const isFromMediaServer = socket.isMediaServer;
    const candidate = data.candidate;
    
    if (!candidate) {
      logger.warn(`Received empty ICE candidate from ${socket.id}`);
      return;
    }
    
    if (isFromMediaServer) {
      // Forward from media server to client
      const targetClientId = data.client_id;
      if (!targetClientId) {
        logger.warn(`Media server sent ICE candidate without client_id`);
        return;
      }
      
      const clientSocket = this.clients.get(targetClientId);
      if (!clientSocket) {
        logger.warn(`Cannot forward ICE: client ${targetClientId} not found`);
        return;
      }
      
      clientSocket.emit('ice', {
        candidate
      });
      
      logger.info(`Forwarded ICE candidate from media server to client ${targetClientId}`);
    } else {
      // Forward from client to media server
      if (this.mediaServers.size === 0) {
        logger.warn(`No media servers to forward ICE candidate from ${socket.id}`);
        return;
      }
      
      // Use the first media server for now
      const mediaServerId = [...this.mediaServers][0];
      const mediaServer = this.clients.get(mediaServerId);
      
      if (!mediaServer) {
        logger.error(`Media server ${mediaServerId} not found in clients map`);
        return;
      }
      
      mediaServer.emit('ice', {
        client_id: socket.id,
        candidate
      });
      
      logger.info(`Forwarded ICE candidate from client ${socket.id} to media server`);
    }
  }

  /**
   * Handle socket disconnection
   * @param {Socket} socket - The Socket.IO socket that disconnected
   * @private
   */
  _handleDisconnection(socket) {
    const socketId = socket.id;
    
    if (socket.isMediaServer) {
      this.mediaServers.delete(socketId);
    }
    
    this.clients.delete(socketId);
    logger.info(`Socket ${socketId} disconnected`);
  }
}

module.exports = SignalingService; 