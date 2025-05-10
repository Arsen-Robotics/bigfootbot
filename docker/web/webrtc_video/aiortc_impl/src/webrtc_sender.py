import asyncio
import fractions
import json
import logging
import os
import time
import traceback
import aiohttp
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer, RTCIceCandidate
import cv2
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger("webrtc_sender")

# Active peer connections
pcs = set()
# Store peer connections by client ID
pc_by_client = {}

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

class CameraVideoTrack(VideoStreamTrack):
    """
    Capture video from a camera for WebRTC streaming.
    """
    
    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self.counter = 0
        self.cap = None
        self.start_camera()
        logger.info(f"Created camera video track using camera ID {camera_id}")
    
    def start_camera(self):
        """Initialize camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                logger.error(f"Failed to open camera {self.camera_id}")
                return False
            
            # Set camera resolution (optional)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            return True
        except Exception as e:
            logger.error(f"Error initializing camera: {e}")
            logger.error(traceback.format_exc())
            return False
    
    async def recv(self):
        """Capture a frame from the camera."""
        if not self.cap or not self.cap.isOpened():
            # If camera isn't working, restart it or return a blank frame
            if not self.start_camera():
                # Return a blank frame with error message
                frame = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(
                    frame,
                    "CAMERA ERROR",
                    (640 // 2 - 100, 480 // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2
                )
            else:
                # Camera restarted, but still need a frame
                ret, frame = self.cap.read()
                if not ret:
                    # Still couldn't get a frame
                    frame = np.zeros((480, 640, 3), np.uint8)
                    cv2.putText(
                        frame,
                        "NO CAMERA FRAME",
                        (640 // 2 - 120, 480 // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2
                    )
        else:
            # Normal camera capture
            ret, frame = self.cap.read()
            if not ret:
                # Failed to get frame
                frame = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(
                    frame,
                    "NO CAMERA FRAME",
                    (640 // 2 - 120, 480 // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
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
            (255, 255, 255),
            2
        )
        
        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        self.counter += 1
        pts = int(self.counter * 90000 / 30)  # 30fps
        video_frame.pts = pts
        video_frame.time_base = fractions.Fraction(1, 90000)
        
        # Target 30fps
        await asyncio.sleep(1/30)
        return video_frame
    
    def __del__(self):
        """Clean up camera resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()

def create_video_track(client_preference=None):
    """
    Create the appropriate video track based on configuration.
    
    Args:
        client_preference: Optional video source preference from the client
    
    Returns:
        VideoStreamTrack: The appropriate video track instance
    """
    # Check environment variables for default configuration
    default_source = os.environ.get("VIDEO_SOURCE", "camera").lower()
    camera_id = int(os.environ.get("CAMERA_ID", "0"))
    
    # Use client preference if provided, otherwise use default
    video_source = client_preference.lower() if client_preference else default_source
    
    if video_source == "test" or video_source == "colorbar" or video_source == "colorbars":
        logger.info("Using color bars test pattern as video source")
        return ColorBarsVideoTrack()
    else:
        logger.info(f"Using camera (ID: {camera_id}) as video source")
        return CameraVideoTrack(camera_id=camera_id)

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
                        logger.info("Sent register_media_server request")
                        
                        # Process incoming messages
                        async for msg in ws:
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                data = json.loads(msg.data)
                                logger.info(f"Received message: {data.get('type')}")
                                
                                if data.get("type") == "offer":
                                    # Process the offer
                                    client_id = data.get("client_id")
                                    sdp = data.get("sdp")
                                    # Get video source preference from client if provided
                                    video_source = data.get("video_source")
                                    
                                    logger.info(f"Received offer from client {client_id}")
                                    if video_source:
                                        logger.info(f"Client requested video source: {video_source}")
                                    
                                    if sdp:
                                        first_line = sdp.split('\n')[0] if '\n' in sdp else sdp[:50]
                                        logger.info(f"Offer SDP first line: {first_line}")
                                    else:
                                        logger.info("Offer SDP: None")
                                    
                                    try:
                                        # Create a list of ICE servers using RTCIceServer
                                        ice_servers = [
                                            RTCIceServer(urls="stun:stun.l.google.com:19302"),
                                            RTCIceServer(urls="stun:stun1.l.google.com:19302"),
                                            RTCIceServer(urls="stun:stun2.l.google.com:19302"),
                                            # Free TURN servers for NAT traversal
                                            RTCIceServer(
                                                urls="turn:standard.relay.metered.ca:80",
                                                username="e7732b9290f5e64733c79636",
                                                credential="873vUZ4RAg01OUqD"
                                            ),
                                            RTCIceServer(
                                                urls="turn:standard.relay.metered.ca:80?transport=tcp",
                                                username="e7732b9290f5e64733c79636",
                                                credential="873vUZ4RAg01OUqD"
                                            ),
                                            RTCIceServer(
                                                urls="turn:standard.relay.metered.ca:443",
                                                username="e7732b9290f5e64733c79636",
                                                credential="873vUZ4RAg01OUqD"
                                            ),
                                            RTCIceServer(
                                                urls="turns:standard.relay.metered.ca:443?transport=tcp",
                                                username="e7732b9290f5e64733c79636",
                                                credential="873vUZ4RAg01OUqD"
                                            )
                                        ]
                                        
                                        # Create the configuration with the ice_servers
                                        config = RTCConfiguration(iceServers=ice_servers)
                                        
                                        # Create peer connection with proper configuration
                                        pc = RTCPeerConnection(config)
                                        
                                        pcs.add(pc)
                                        # Store PC by client ID for ICE candidate handling
                                        pc_by_client[client_id] = pc
                                        logger.info(f"Created RTCPeerConnection for {client_id}")
                                        
                                        # Add event handlers
                                        @pc.on("connectionstatechange")
                                        async def on_connectionstatechange():
                                            logger.info(f"Connection state for {client_id}: {pc.connectionState}")
                                            if pc.connectionState == "failed" or pc.connectionState == "closed":
                                                logger.info(f"Closing connection for {client_id}")
                                                if pc in pcs:
                                                    pcs.discard(pc)
                                                if client_id in pc_by_client:
                                                    del pc_by_client[client_id]
                                                await pc.close()
                                        
                                        # Monitor ICE gathering state
                                        @pc.on("icegatheringstatechange")
                                        async def on_icegatheringstatechange():
                                            logger.info(f"ICE gathering state for {client_id}: {pc.iceGatheringState}")
                                            
                                        # Log ICE candidates generated by the server
                                        @pc.on("icecandidate")
                                        async def on_icecandidate(candidate):
                                            if candidate:
                                                logger.info(f"Generated ICE candidate: {candidate.candidate}")
                                                # You could also forward this to the client if needed
                                        
                                        # Add a video track based on the configured source
                                        video_track = create_video_track(client_preference=video_source)
                                        pc.addTrack(video_track)
                                        logger.info(f"Added video track for {client_id}")
                                        
                                        # Set the remote description
                                        try:
                                            await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
                                            logger.info(f"Set remote description for {client_id}")
                                        except Exception as e:
                                            logger.error(f"Error setting remote description: {e}")
                                            logger.error(traceback.format_exc())
                                            continue
                                        
                                        # Create an answer
                                        try:
                                            answer = await pc.createAnswer()
                                            await pc.setLocalDescription(answer)
                                            logger.info(f"Created answer for {client_id}")
                                            if pc.localDescription and pc.localDescription.sdp:
                                                first_line = pc.localDescription.sdp.split('\n')[0] if '\n' in pc.localDescription.sdp else pc.localDescription.sdp[:50]
                                                logger.info(f"Answer SDP first line: {first_line}")
                                            else:
                                                logger.info("Answer SDP: None")
                                        except Exception as e:
                                            logger.error(f"Error creating answer: {e}")
                                            logger.error(traceback.format_exc())
                                            continue
                                        
                                        # Send answer back to signaling server
                                        try:
                                            await ws.send_json({
                                                "type": "answer",
                                                "client_id": client_id,
                                                "sdp": pc.localDescription.sdp
                                            })
                                            logger.info(f"Sent answer to signaling server for {client_id}")
                                        except Exception as e:
                                            logger.error(f"Error sending answer: {e}")
                                            logger.error(traceback.format_exc())
                                    except Exception as e:
                                        logger.error(f"Error processing offer for {client_id}: {e}")
                                        logger.error(traceback.format_exc())
                                
                                elif data.get("type") == "ice":
                                    # Handle ICE candidate
                                    client_id = data.get("client_id")
                                    candidate = data.get("candidate")
                                    
                                    if client_id in pc_by_client and candidate:
                                        pc = pc_by_client[client_id]
                                        logger.info(f"Received ICE candidate for {client_id}: {str(candidate)[:50]}...")
                                        try:
                                            # Check if candidate is a dictionary (WebRTC object)
                                            if isinstance(candidate, dict) and 'candidate' in candidate:
                                                # Extract the candidate string and other properties
                                                candidate_str = candidate.get('candidate')
                                                sdp_mid = candidate.get('sdpMid')
                                                sdp_m_line_index = candidate.get('sdpMLineIndex')
                                                
                                                # Parse the candidate string to extract all required fields
                                                # Format: candidate:foundation component protocol priority ip port typ type ...
                                                logger.info(f"Extracted candidate: {candidate_str[:50]}...")
                                                try:
                                                    parts = candidate_str.split()
                                                    if len(parts) >= 8:
                                                        foundation = parts[0].split(':')[1]  # Remove 'candidate:' prefix
                                                        component = int(parts[1])
                                                        protocol = parts[2]
                                                        priority = int(parts[3])
                                                        ip = parts[4]
                                                        port = int(parts[5])
                                                        # parts[6] is 'typ'
                                                        candidate_type = parts[7]
                                                        
                                                        # Create a properly formatted aiortc RTCIceCandidate object
                                                        ice = RTCIceCandidate(
                                                            foundation=foundation,
                                                            component=component,
                                                            protocol=protocol,
                                                            priority=priority,
                                                            ip=ip,
                                                            port=port,
                                                            type=candidate_type,
                                                            sdpMid=sdp_mid,
                                                            sdpMLineIndex=sdp_m_line_index
                                                        )
                                                        logger.info(f"Created ICE candidate: foundation={foundation}, type={candidate_type}, ip={ip}, port={port}")
                                                        await pc.addIceCandidate(ice)
                                                    else:
                                                        logger.warning(f"Invalid candidate format: {candidate_str}")
                                                        continue
                                                except Exception as e:
                                                    logger.error(f"Error parsing ICE candidate: {e}")
                                                    logger.error(traceback.format_exc())
                                            else:
                                                # Try to use the candidate directly
                                                await pc.addIceCandidate(candidate)
                                            
                                            logger.info(f"Added ICE candidate for {client_id}")
                                        except Exception as e:
                                            logger.error(f"Error adding ICE candidate: {e}")
                                            logger.error(traceback.format_exc())
                                    else:
                                        logger.info(f"Ignoring ICE candidate for unknown client {client_id}")
                            
                            elif msg.type == aiohttp.WSMsgType.CLOSED:
                                logger.info("WebSocket connection closed")
                                break
                            elif msg.type == aiohttp.WSMsgType.ERROR:
                                logger.error(f"WebSocket error: {ws.exception()}")
                                break
                except Exception as e:
                    logger.error(f"Connection to signaling server failed: {e}")
                    logger.error(traceback.format_exc())
                    await asyncio.sleep(5)  # Wait before retrying
    except Exception as e:
        logger.error(f"Error in signaling connection: {e}")
        logger.error(traceback.format_exc())

async def cleanup():
    """Close all peer connections."""
    logger.info(f"Cleaning up {len(pcs)} peer connections")
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    pc_by_client.clear()

async def main():
    """Run the WebRTC sender."""
    try:
        # Connect to signaling server
        await connect_to_signaling_server()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down")
    finally:
        await cleanup()

if __name__ == "__main__":
    # Run the WebRTC sender
    logger.info("Starting WebRTC sender")
    asyncio.run(main())
    logger.info("WebRTC sender stopped") 