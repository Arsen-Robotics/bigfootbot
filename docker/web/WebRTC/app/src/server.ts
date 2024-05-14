import express, { Application } from "express";
import { Server as SocketIOServer } from "socket.io";
import { createServer, Server as HTTPServer } from "http";
 
export class Server { // export means that this class can be imported in other files
  private httpServer: HTTPServer;
  private app: Application;
  private io: SocketIOServer;
  
  private readonly DEFAULT_PORT = 3000; // server listens on port 3000
 
  constructor() {
    this.app = express();
    this.httpServer = createServer(this.app);
    this.io = new SocketIOServer(this.httpServer);
    //this.io = socketIO(this.httpServer);
    //this.initialize();
  
    this.handleRoutes();
    this.handleSocketConnection();
  }
 
  /*private initialize(): void {
    this.app = express();
    this.httpServer = createServer(this.app);
    this.io = socketIO(this.httpServer);
  }*/
 
  // Define a route handler for the default home page
  private handleRoutes(): void {
    this.app.get("/", (req, res) => {
      res.send(`<h1>Hello World</h1>`); 
    });
  }
 
  private handleSocketConnection(): void {
    this.io.on("connection", socket => {
      console.log("Socket connected.");
    });
  }
 
  public listen(callback: (port: number) => void): void {
    this.httpServer.listen(this.DEFAULT_PORT, () =>
      callback(this.DEFAULT_PORT)
    );
  }
}