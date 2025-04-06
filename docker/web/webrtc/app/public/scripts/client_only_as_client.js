/* 

This script handles the setup of the WebRTC connection, the exchange of signaling messages with the server, 
the reception and display of the remote media stream. This facilitates real-time video streaming from 
the robot to the web client (robot operator's browser).

High-level overview of the WebRTC connection process between the client (robot operator's browser) and server (robot):

1. Client Connects: The client connects to the server via Socket.io.
2. Server Sends Offer: The server automatically generates and sends an offer to the client upon connection.
3. Client Handles Offer: The client sets the remote description with the received offer and creates an answer.
4. Client Sends Answer: The client sends the answer back to the server.
5. ICE Candidate Exchange: Both the client and server exchange ICE candidates to establish a reliable connection.

*/

// Establish connection to the server via Socket.io
const socket = io();

// Get the video element from the HTML
const remoteVideo = document.getElementById('remoteVideo');

// Create a new RTCPeerConnection with STUN server for ICE candidates
// It is used to establish a connection between the two peers (in our case one peer is the client [operator's browser]
// and the other peer is the server [server acts as signaling server and the peer that sends the video stream])
let pc = new RTCPeerConnection({
  iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
});

// Handle ICE candidates by sending them to the server
pc.onicecandidate = (event) => {
  if (event.candidate) {
    socket.emit('candidate', event.candidate);
  }
};

// When a track is received, set it as the source for the video element
// Track is the media stream that is sent by the other peer (in our case the server)
pc.ontrack = (event) => {
  remoteVideo.srcObject = event.streams[0];
};

// Handle the offer from the server
socket.on('offer', async (offer) => { // async is used to make the function asynchronous. Offer is the promise that is resolved.
  // Set the remote description to the received offer
  await pc.setRemoteDescription(new RTCSessionDescription(offer));
  
  // Create an answer to the offer
  const answer = await pc.createAnswer(); // pauses execution until the promise returned by createAnswer resolves. 
  
  // Set the local description to the created answer
  await pc.setLocalDescription(answer);
  
  // Send the answer back to the server
  socket.emit('answer', answer);
});

// Handle the answer from the server (in the case when the client is the one who sends the offer)
// NOTE In our case, where the camera stream goes from the robot to the client, the client does not send an offer. 
// The server (robot) initiates the WebRTC connection by sending an offer to the client. 
/*socket.on('answer', async (answer) => {
  // Set the remote description to the received answer
  await pc.setRemoteDescription(new RTCSessionDescription(answer));
});*/

// Handle the ICE candidate from the server
socket.on('candidate', async (candidate) => {
  // Add the received ICE candidate to the peer connection
  await pc.addIceCandidate(new RTCIceCandidate(candidate));
}); 