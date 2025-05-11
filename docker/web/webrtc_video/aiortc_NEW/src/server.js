const http = require('http');
const express = require('express');
const { Server } = require('socket.io');
const cors = require('cors');
const { PORT } = require('./config');
const logger = require('./utils/logger');
const SignalingService = require('./services/SignalingService');

// Create Express app
const app = express();

// Enable CORS for all routes
app.use(cors());

// Basic health check endpoint
app.get('/health', (req, res) => {
  res.status(200).send({ status: 'ok' });
});

// Create HTTP server
const server = http.createServer(app);

// Create Socket.IO server with CORS configuration
const io = new Server(server, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST']
  }
});

// Create signaling service and pass Socket.IO instance
const signalingService = new SignalingService(io);

// Start the server
server.listen(PORT, () => {
  logger.info(`Signaling server running on port ${PORT}`);
});

// Handle graceful shutdown
const gracefulShutdown = () => {
  logger.info('Received shutdown signal, closing server...');
  
  // Close Socket.IO connections
  io.close(() => {
    logger.info('Socket.IO server closed');
    
    // Close HTTP server
    server.close(() => {
      logger.info('HTTP server closed');
      process.exit(0);
    });
  });
  
  // If server doesn't close in 10 seconds, force exit
  setTimeout(() => {
    logger.error('Could not close connections in time, forcefully shutting down');
    process.exit(1);
  }, 10000);
};

// Listen for termination signals
process.on('SIGTERM', gracefulShutdown);
process.on('SIGINT', gracefulShutdown);

module.exports = { app, server, io }; 