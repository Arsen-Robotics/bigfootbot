// Load environment variables
require('dotenv').config();

// Get environment variables with defaults
const PORT = process.env.PORT || 8765;
const NODE_ENV = process.env.NODE_ENV || 'development';
const TURN_USERNAME = process.env.TURN_USERNAME || 'e7732b9290f5e64733c79636';
const TURN_CREDENTIAL = process.env.TURN_CREDENTIAL || '873vUZ4RAg01OUqD';

// ICE server configuration
const iceServers = [
  // Free STUN servers
  { urls: 'stun:stun.l.google.com:19302' },
  { urls: 'stun:stun1.l.google.com:19302' },
  { urls: 'stun:stun2.l.google.com:19302' },
  { urls: 'stun:stun3.l.google.com:19302' },
  { urls: 'stun:stun4.l.google.com:19302' },
  
  // TURN servers for NAT traversal
  {
    urls: 'turn:standard.relay.metered.ca:80',
    username: TURN_USERNAME,
    credential: TURN_CREDENTIAL
  },
  {
    urls: 'turn:standard.relay.metered.ca:80?transport=tcp',
    username: TURN_USERNAME,
    credential: TURN_CREDENTIAL
  },
  {
    urls: 'turn:standard.relay.metered.ca:443',
    username: TURN_USERNAME,
    credential: TURN_CREDENTIAL
  },
  {
    urls: 'turns:standard.relay.metered.ca:443?transport=tcp',
    username: TURN_USERNAME,
    credential: TURN_CREDENTIAL
  }
];

module.exports = {
  PORT,
  NODE_ENV,
  iceServers,
  isDev: NODE_ENV === 'development'
}; 