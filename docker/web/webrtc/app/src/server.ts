import express, { Application } from "express";
import { Server as SocketIOServer } from "socket.io";
import { createServer, Server as HTTPServer } from "http";
import path from "path";

// Main server class
// As a signaling server, it is responsible for handling WebRTC signaling and ICE candidate exchange tasks
export class Server {
  private app: Application;
  private httpServer: HTTPServer;
  private io: SocketIOServer;

  private readonly DEFAULT_PORT = 3000;

  constructor() {
    // Initialize Express application and HTTP server
    this.app = express();
    this.httpServer = createServer(this.app);
    // Initialize Socket.IO for real-time signaling
    this.io = new SocketIOServer(this.httpServer);

    // Configure app and set up routes
    this.configureApp();
    this.configureRoutes();
    this.handleSocketConnection();
  }

  // Serve static files from the 'public' directory
  private configureApp(): void {
    this.app.use(express.static(path.join(__dirname, "../public")));
  }

  // Define routes for the server
  private configureRoutes(): void {
    this.app.get("/", (req, res) => {
      res.send(`<h1>Hello World</h1>`);
    });
  }

  // Handle WebSocket connections for signaling
  private handleSocketConnection(): void {
    this.io.on("connection", (socket) => {
      console.log("Client connected");

      // Relay an offer from one client to all others
      socket.on("offer", (offer) => {
        socket.broadcast.emit("offer", offer);
      });

      // Relay an answer from one client to all others
      socket.on("answer", (answer) => {
        socket.broadcast.emit("answer", answer);
      });

      // Relay ICE candidates from one client to all others
      socket.on("candidate", (candidate) => {
        socket.broadcast.emit("candidate", candidate);
      });

      // Handle client disconnection
      socket.on("disconnect", () => {
        console.log("Client disconnected");
      });
    });
  }

  // Start the HTTP server and listen on the specified port
  /*public listen(callback: (port: number) => void): void {
    this.httpServer.listen(this.DEFAULT_PORT, () => callback(this.DEFAULT_PORT));
  }*/

  public listen(callback: (port: number) => void): void {
    this.httpServer.listen(this.DEFAULT_PORT, "0.0.0.0", () => callback(this.DEFAULT_PORT));
  }
  
}