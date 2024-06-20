// Install with 'npm i <module>' & Import necessary modules
// `npm i` installs the module locallu in the node_modules folder and adds it to the dependencies in package.json
import express, { Application } from "express";
import { Server as SocketIOServer } from "socket.io";
import { createServer, Server as HTTPServer } from "http";
import path from "path";

import { RTCPeerConnection, RTCSessionDescription, RTCIceCandidate } from "webrtc";
 
export class Server { // export means that this class can be imported in other files
  private app: Application;
  private httpServer: HTTPServer;
  private io: SocketIOServer;
  
  private readonly DEFAULT_PORT = 3000; // server listens on port 3000
 
  constructor() {
    // Initialize Express application and HTTP server
    this.app = express();
    this.httpServer = createServer(this.app);
    // Initialize Socket.io server
    this.io = new SocketIOServer(this.httpServer);    
    
    this.configureApp();
    this.configureRoutes();
    this.handleSocketConnection();
  }
   
  // Tell express, which static files you want to serve.
  // In this case, we want to serve the files in the public folder.
  // So typing localhost:3000/index.html (or localhost:3000) will serve the index.html file in the public folder.
  private configureApp(): void {
    this.app.use(express.static(path.join(__dirname, "../public")));
  }
 
  /*private initialize(): void {
    this.app = express();
    this.httpServer = createServer(this.app);
    this.io = socketIO(this.httpServer);
  }*/
 
  private configureRoutes(): void {
    this.app.get("/", (req, res) => { // When you go to localhost:3000, you will see "Hello World"
      res.send(`<h1>Hello World</h1>`);  // But when static files are served, you will see the index.html file in the public folder.
    });

    this.app.get("/test", (req, res) => { // when you go to localhost:3000/test, you will see "Hello test"
      res.send(`<h1>Hello test</h1>`); 
    });
  }

  /**
   * Sets up event listeners that handle various WebRTC signaling and ICE candidate exchange tasks 
   * for each new client connection.
   * 
   * socket.io is used as signaling server (to exchange information between two devices about video/audio streams)
   * So signaling server is this Node.js server (server.ts) and signaling client is the robot operator's browser. 
   * In our case the signaling server is also a WebRTC peer. One peer is the robot (this server) and the other peer is 
   * the robot operator (browser).
   * Operator's browser connect to the signaling server using socket.io (WebSockets or HTTP long polling 
   * [in case WebSocket connection cannot be established]).
   */
  private handleSocketConnection(): void {
    this.io.on("connection", (socket) => { // When a client connects to the server, this function is called.
      console.log("Client connected");

      // Create a new RTCPeerConnection with STUN server configuration
      // Becasue our server not only a signaling server but it is also a WebRTC peer.
      const pc = new RTCPeerConnection({
        iceServers: [{ urls: "stun:stun.l.google.com:19302" }]
      });

      // Create and send offer to the client
      const createOffer = async () => {
        const offer = await pc.createOffer(); // Create an offer
        await pc.setLocalDescription(offer); // Set the local description (SDP)
        socket.emit("offer", offer); // Send the offer to the client
      };

      createOffer(); // Generate and send offer on connection

      // Handle incoming answer from the client
      socket.on("answer", async (answer) => {
        await pc.setRemoteDescription(new RTCSessionDescription(answer)); // Set the remote description
      });

      // Send ICE candidates to the client
      // onicecandidate event is triggered when an ICE candidate has been found.
      // stun server is used to find the public IP address of the client
      pc.onicecandidate = (event) => {
        if (event.candidate) {
          socket.emit("candidate", event.candidate);
        }
      }; 
      
      // Handle incoming ICE candidates from the client
      // ICE candidates are used to help clients find the best connection path to each other.
      socket.on("candidate", async (candidate) => {
        await pc.addIceCandidate(new RTCIceCandidate(candidate)); // Add the ICE candidate
      });            
                           
      // Handles the 'disconnect' event which occurs when a client disconnects from the server.
      socket.on("disconnect", () => {
        console.log("Socket disconnected");
      });
    });
  }
 
  /**
   * Starts the HTTP server and listens for incoming connections on the specified port.
   * @param callback - A callback function that is called once the server starts listening.
   * The callback receives the port number as an argument.
   */
  public listen(callback: (port: number) => void): void {
    this.httpServer.listen(this.DEFAULT_PORT, () =>
      callback(this.DEFAULT_PORT)
    );
  }
}