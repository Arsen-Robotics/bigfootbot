import asyncio
import websockets
import json

# Store connected clients
clients = set()

SERVER_IP = "0.0.0.0"
SERVER_PORT = 8765

async def signaling_handler(websocket):
    clients.add(websocket)
    print("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)
            print(f"Received: {data}")

            # Forward messages to all other clients
            for client in clients:
                if client != websocket:
                    await client.send(json.dumps(data))

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        clients.remove(websocket)

async def main():
    server = await websockets.serve(signaling_handler, SERVER_IP, SERVER_PORT)
    print(f"Signaling server running on ws://{SERVER_IP}:{SERVER_PORT}")
    await server.wait_closed()

asyncio.run(main())
