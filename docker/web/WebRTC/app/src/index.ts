// To run our web server, we need to create an instance of the Server class and call the listen method.
// To run the server, call one of the following commands in the terminal:
// - npm run start 
// - npm run dev (if you want to run the server in development mode)
// In the terminal you will see the message "Server is listening on http://localhost:3000"
// Then open in a browser the address http://localhost:3000 and you will see the message "Hello World"

// Install with 'npm i <module>' & Import necessary modules

// Import the Server class from the server module (server.ts)
import { Server } from "./server";

// Create an instance of the Server class
const server = new Server();

/**
 * Start the server and listen on the specified port (port is specified in the server.ts file).
 * The listen method accepts a callback function which is executed 
 * once the server starts listening. The callback function receives 
 * the port number as an argument and logs a message indicating 
 * that the server is running and on which URL it can be accessed.
 * 
 * port => { } is an arrow function that is passed as a callback to the listen method
 */ 
server.listen(port => {
  console.log(`Server is listening on http://localhost:${port}`);
});