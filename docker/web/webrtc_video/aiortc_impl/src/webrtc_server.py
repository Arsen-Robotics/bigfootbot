import asyncio
import fractions
import json
import logging
import os
import time
import traceback
import aiohttp
from aiohttp import web
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import cv2
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger("webrtc")

# Active peer connections
pcs = set()

class ColorBarsVideoTrack(VideoStreamTrack):
    """
    Generate color bars as a video source for WebRTC.
    """
    
    def __init__(self):
        super().__init__()
        self.counter = 0
        logger.info("Created color bars test pattern video track")
    
    async def recv(self):
        """Generate a frame with color bars."""
        # Create color bars pattern
        height, width = 480, 640
        frame = np.zeros((height, width, 3), np.uint8)
        
        # Draw colored bars
        bar_width = width // 7
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (255, 0, 255),  # Magenta
            (255, 255, 255)  # White
        ]
        
        for i, color in enumerate(colors):
            x = i * bar_width
            frame[:, x:x+bar_width] = color
        
        # Add text
        cv2.putText(
            frame,
            "TEST PATTERN",
            (width // 2 - 100, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            2
        )
        
        # Add timestamp
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(
            frame,
            timestamp,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            2
        )
        
        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        self.counter += 1
        pts = int(self.counter * 90000 / 30)  # 30fps
        video_frame.pts = pts
        video_frame.time_base = fractions.Fraction(1, 90000)
        
        # Simulate 30fps
        await asyncio.sleep(1/30)
        return video_frame

async def index(request):
    """Serve the index.html file."""
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    content = open(os.path.join(root_dir, "static", "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):
    """Serve the client.js file."""
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    content = open(os.path.join(root_dir, "static", "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    """Process WebRTC offers directly."""
    try:
        params = await request.json()
        logger.info(f"Received WebRTC offer from client: {request.remote}")
        
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        # Create a new peer connection
        pc = RTCPeerConnection()
        pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"Connection state is {pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await pc.close()
                pcs.discard(pc)
        
        # Add the video track (color bars)
        video = ColorBarsVideoTrack()
        pc.addTrack(video)
        logger.info("Added color bars video track to peer connection")
        
        # Set the remote description (client's offer)
        await pc.setRemoteDescription(offer)
        logger.info("Set remote description (client's offer)")
        
        # Create an answer and set it as local description
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        logger.info("Created and set local description (our answer)")
        
        # Return the answer to the client
        logger.info("Sending answer back to client")
        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })
        )
    except Exception as e:
        logger.error(f"Error handling offer: {e}")
        logger.error(traceback.format_exc())
        return web.Response(
            status=500,
            content_type="application/json",
            text=json.dumps({"error": str(e)})
        )

async def on_shutdown(app):
    """Close all peer connections on shutdown."""
    # Close all peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

async def connect_to_signaling_server():
    """Connect to the signaling server and handle WebRTC signaling."""
    try:
        # Connect to the signaling server to register as a media server
        signaling_url = os.environ.get("SIGNALING_URL", "ws://localhost:8765")
        logger.info(f"Connecting to signaling server at {signaling_url}")
        
        async with aiohttp.ClientSession() as session:
            # Keep trying to connect to the signaling server
            while True:
                try:
                    async with session.ws_connect(signaling_url) as ws:
                        logger.info("Connected to signaling server")
                        
                        # Register as a media server
                        await ws.send_json({"type": "register_media_server"})
                        
                        # Process incoming messages
                        async for msg in ws:
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                data = json.loads(msg.data)
                                if data.get("type") == "offer":
                                    # Process the offer
                                    client_id = data.get("client_id")
                                    sdp = data.get("sdp")
                                    logger.info(f"Received offer from client {client_id} via signaling server")
                                    
                                    # Create a peer connection
                                    pc = RTCPeerConnection()
                                    pcs.add(pc)
                                    
                                    # Add a video track
                                    pc.addTrack(ColorBarsVideoTrack())
                                    
                                    # Set the remote description
                                    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
                                    
                                    # Create an answer
                                    answer = await pc.createAnswer()
                                    await pc.setLocalDescription(answer)
                                    
                                    # Send answer back to signaling server
                                    await ws.send_json({
                                        "type": "answer",
                                        "client_id": client_id,
                                        "sdp": pc.localDescription.sdp
                                    })
                                    logger.info(f"Sent answer to client {client_id} via signaling server")
                            elif msg.type == aiohttp.WSMsgType.CLOSED:
                                logger.info("WebSocket connection closed")
                                break
                            elif msg.type == aiohttp.WSMsgType.ERROR:
                                logger.error(f"WebSocket error: {ws.exception()}")
                                break
                except Exception as e:
                    logger.error(f"Connection to signaling server failed: {e}")
                    await asyncio.sleep(5)  # Wait before retrying
    except Exception as e:
        logger.error(f"Error in signaling connection: {e}")
        logger.error(traceback.format_exc())

def create_app():
    """Create and configure the application."""
    app = web.Application()
    
    # Add routes
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    
    # Add static files
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    app.router.add_static("/static", os.path.join(root_dir, "static"))
    
    # Add shutdown handler
    app.on_shutdown.append(on_shutdown)
    
    # Start background task to connect to signaling server
    asyncio.ensure_future(connect_to_signaling_server())
    
    return app

if __name__ == "__main__":
    # Run the WebRTC server
    app = create_app()
    web.run_app(app, host="0.0.0.0", port=8080)
    logger.info("WebRTC server stopped") 