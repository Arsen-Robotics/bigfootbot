import asyncio
import json
import websockets
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GLib, GstWebRTC, GstSdp

class WebRTCRecv:
    def __init__(self):
        self.SIGNALING_SERVER = 'ws://0.0.0.0:8765'
        self.websocket = None
        self.webrtcbin = None
        self.pipeline = None

    async def connect_to_server(self):
        """Establish WebSocket connection to the signaling server."""
        try:
            self.websocket = await websockets.connect(self.SIGNALING_SERVER)
            print("Connected to signaling server")
        except Exception as e:
            print(f"Failed to connect to signaling server: {e}")
