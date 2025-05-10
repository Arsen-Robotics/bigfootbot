import asyncio
import json
import logging
import uuid
import traceback
from aiohttp import web, WSMsgType
import aiohttp_cors

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger("signaling")

class SignalingServer:
    """Simple WebRTC signaling server that only handles WebSocket connections."""
    
    def __init__(self):
        """Initialize the signaling server."""
        self.clients = {}  # Dictionary of connected clients by client_id
        self.media_servers = []  # List of WebSocket connections to media servers
        self.ice_servers = [
            {"urls": "stun:stun.l.google.com:19302"},
            {"urls": "stun:stun1.l.google.com:19302"},
            {"urls": "stun:stun2.l.google.com:19302"},
            {"urls": "stun:stun3.l.google.com:19302"},
            {"urls": "stun:stun4.l.google.com:19302"},
            # Free TURN servers for NAT traversal
            {
                "urls": "turn:openrelay.metered.ca:80",
                "username": "openrelayproject",
                "credential": "openrelayproject"
            },
            {
                "urls": "turn:openrelay.metered.ca:443",
                "username": "openrelayproject",
                "credential": "openrelayproject"
            }
        ]
        logger.info(f"Signaling server initialized with {len(self.ice_servers)} ICE servers")
    
    async def websocket_handler(self, request):
        """Handle WebSocket connections for signaling."""
        logger.info(f"New WebSocket connection from {request.remote}")
        
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        # Generate a unique client ID
        client_id = str(uuid.uuid4())
        logger.info(f"Client {client_id} connected from {request.remote}")
        
        # This WebSocket could be from a client or a media server
        is_media_server = False
        
        try:
            # Handle incoming messages
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        message_type = data.get("type", "unknown")
                        logger.info(f"Received {message_type} from {client_id}")
                        
                        # Media server registration
                        if message_type == "register_media_server":
                            logger.info(f"Registering client {client_id} as media server")
                            self.media_servers.append(ws)
                            is_media_server = True
                            
                            # Send confirmation
                            await ws.send_json({
                                "type": "registered_media_server",
                                "id": client_id
                            })
                        
                        # Media server answer (to forward to client)
                        elif message_type == "answer" and is_media_server:
                            target_client_id = data.get("client_id")
                            logger.info(f"Received answer from media server for client {target_client_id}")
                            if target_client_id in self.clients:
                                client_ws = self.clients[target_client_id]
                                try:
                                    # Extract first line of SDP for logging
                                    sdp = data.get("sdp", "")
                                    if sdp:
                                        first_line = sdp.split('\n')[0] if '\n' in sdp else sdp[:50]
                                        logger.info(f"Answer SDP first line: {first_line}")
                                    else:
                                        logger.info("Answer SDP: None")
                                    
                                    await client_ws.send_json({
                                        "type": "answer",
                                        "sdp": data.get("sdp")
                                    })
                                    logger.info(f"Forwarded answer from media server to client {target_client_id}")
                                except Exception as e:
                                    logger.error(f"Error forwarding answer to client {target_client_id}: {e}")
                                    logger.error(traceback.format_exc())
                            else:
                                logger.warning(f"Client {target_client_id} not found for answer forwarding")
                                logger.warning(f"Currently connected clients: {list(self.clients.keys())}")
                        
                        # Client registration
                        elif message_type == "register" and not is_media_server:
                            # Store the client connection
                            self.clients[client_id] = ws
                            
                            # Send ICE servers configuration
                            await ws.send_json({
                                "type": "config",
                                "iceServers": self.ice_servers
                            })
                            logger.info(f"Sent ICE server config to {client_id}")
                            
                            # Send confirmation
                            await ws.send_json({
                                "type": "registered",
                                "id": client_id
                            })
                            logger.info(f"Client {client_id} registered")
                        
                        # Received SDP offer - forward to WebRTC server if available
                        elif message_type == "offer" and not is_media_server:
                            logger.info(f"Received offer from {client_id}")
                            # Extract first line of SDP for logging
                            sdp = data.get("sdp", "")
                            if sdp:
                                first_line = sdp.split('\n')[0] if '\n' in sdp else sdp[:50]
                                logger.info(f"Offer SDP first line: {first_line}")
                            else:
                                logger.info(f"Offer SDP: None")
                            
                            # Check if we have any registered media servers
                            if self.media_servers:
                                logger.info(f"Found {len(self.media_servers)} media servers to forward offer to")
                                # Forward to the first media server for now (can implement load balancing later)
                                media_server = self.media_servers[0]
                                try:
                                    # Get video source preference if specified by the client
                                    video_source = data.get("video_source", "camera")
                                    
                                    # Forward the offer with video source preference to media server
                                    await media_server.send_json({
                                        "type": "offer",
                                        "client_id": client_id,
                                        "sdp": sdp,
                                        "video_source": video_source
                                    })
                                    logger.info(f"Forwarded offer from client {client_id} to media server with video source: {video_source}")
                                except Exception as e:
                                    logger.error(f"Error forwarding offer to media server: {e}")
                                    logger.error(traceback.format_exc())
                                    # Fall back to SDP modification approach
                                    await self.handle_offer_fallback(ws, client_id, data.get("sdp"))
                            else:
                                logger.warning("No media servers available to forward offer to")
                                # No media servers, use the fallback approach
                                await self.handle_offer_fallback(ws, client_id, data.get("sdp"))
                        
                        # ICE Candidate
                        elif message_type == "ice" and not is_media_server:
                            logger.info(f"Received ICE candidate from {client_id}")
                            # Acknowledge receipt
                            await ws.send_json({
                                "type": "ack",
                                "for": "ice"
                            })
                            
                            # Forward to media server if available
                            if self.media_servers:
                                media_server = self.media_servers[0]
                                try:
                                    # Simplify - just pass the entire candidate object
                                    await media_server.send_json({
                                        "type": "ice",
                                        "client_id": client_id,
                                        "candidate": data.get("candidate")
                                    })
                                    logger.info(f"Forwarded ICE candidate from client {client_id} to media server")
                                except Exception as e:
                                    logger.error(f"Error forwarding ICE candidate to media server: {e}")
                        
                        # Disconnect request
                        elif message_type == "disconnect":
                            logger.info(f"Client {client_id} requested disconnect")
                            break
                    
                    except json.JSONDecodeError:
                        logger.warning(f"Received invalid JSON from {client_id}")
                    except Exception as e:
                        logger.error(f"Error processing message: {e}")
                        logger.error(traceback.format_exc())
                
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        
        finally:
            # Clean up when client disconnects
            if not is_media_server and client_id in self.clients:
                del self.clients[client_id]
                logger.info(f"Client {client_id} disconnected")
            elif is_media_server and ws in self.media_servers:
                self.media_servers.remove(ws)
                logger.info(f"Media server {client_id} disconnected")
        
        return ws
    
    async def handle_offer_fallback(self, ws, client_id, sdp):
        """Fallback method to handle offers when no media server is available."""
        try:
            logger.info(f"Using fallback method to generate answer for {client_id}")
            
            # Fix the SDP by modifying the setup attribute
            modified_sdp = []
            for line in sdp.split('\r\n'):
                if line.startswith('a=setup:'):
                    if line == 'a=setup:actpass':
                        line = 'a=setup:active'
                    elif line == 'a=setup:active':
                        line = 'a=setup:passive'
                modified_sdp.append(line)
            
            modified_sdp_string = '\r\n'.join(modified_sdp)
            logger.info(f"Modified SDP answer to fix setup attribute")
            
            await ws.send_json({
                "type": "answer",
                "sdp": modified_sdp_string
            })
            logger.info(f"Sent fallback answer to {client_id}")
        except Exception as e:
            logger.error(f"Error generating fallback answer: {e}")
            logger.error(f"Error details: {traceback.format_exc()}")
    
    async def broadcast(self, message):
        """Broadcast a message to all connected clients."""
        logger.info(f"Broadcasting message to {len(self.clients)} clients")
        for client_id, ws in list(self.clients.items()):
            try:
                await ws.send_json(message)
            except Exception as e:
                logger.warning(f"Failed to send to {client_id}: {e}")
                # Clean up disconnected client
                if client_id in self.clients:
                    del self.clients[client_id]

def create_app():
    """Create and configure the application."""
    app = web.Application()
    
    # Set up CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods=["GET", "POST", "OPTIONS"]
        )
    })
    
    # Create signaling server
    signaling = SignalingServer()
    
    # Add routes with CORS support
    resource = cors.add(app.router.add_resource("/"))
    cors.add(resource.add_route("GET", signaling.websocket_handler))
    
    # Store the signaling server in the app
    app["signaling"] = signaling
    
    return app

if __name__ == "__main__":
    # Set up a proper logging configuration for access logs
    access_logger = logging.getLogger('aiohttp.access')
    access_logger.setLevel(logging.INFO)
    
    # Run the signaling server
    app = create_app()
    logger.info("Starting signaling server on 0.0.0.0:8765")
    web.run_app(app, host="0.0.0.0", port=8765, access_log=access_logger)
    logger.info("Signaling server stopped") 