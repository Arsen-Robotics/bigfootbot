import express, { Application } from "express";
import { Server as SocketIOServer } from "socket.io";
import { createServer, Server as HTTPServer } from "http";
import path from "path";
 
export class Server { // export means that this class can be imported in other files
  private app: Application;
  private httpServer: HTTPServer;
  private io: SocketIOServer;
  
  private readonly DEFAULT_PORT = 3000; // server listens on port 3000
 
  constructor() {
    this.app = express();
    this.httpServer = createServer(this.app);
    this.io = new SocketIOServer(this.httpServer);
    //this.io = socketIO(this.httpServer);
    //this.initialize();
    
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