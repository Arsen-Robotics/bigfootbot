import asyncio
import websockets
import json

async def send_json_message():
    uri = "ws://localhost:8765"
    message = "Test message"
    
    async with websockets.connect(uri) as websocket:
        await websocket.send(json.dumps(message))
        response = await websocket.recv()
        print(f"Received: {response}")

asyncio.run(send_json_message())