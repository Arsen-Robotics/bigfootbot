import asyncio
import websockets
import json

# Store connected clients
clients = set()

async def signaling_handler(websocket, path):
    clients.add(websocket)
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
    server = await websockets.serve(signaling_handler, "0.0.0.0", 8765)
    print("Signaling server running on ws://0.0.0.0:8765")
    await server.wait_closed()

asyncio.run(main())
