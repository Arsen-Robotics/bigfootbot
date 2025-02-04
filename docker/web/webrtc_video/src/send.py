import asyncio
import json
import websockets
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GLib

SIGNALING_SERVER = 'ws://127.0.0.1:8765'
websocket = None

async def connect_to_server():
    """Establish WebSocket connection to the signaling server."""
    global websocket
    try:
        websocket = await websockets.connect(SIGNALING_SERVER)
        print("Connected to signaling server")
    except Exception as e:
        print(f"Failed to connect to signaling server: {e}")

def on_negotiation_needed(webrtcbin):
    """Triggered when WebRTC negotiation is needed."""
    print("Negotiation needed, creating offer...")
    promise = Gst.Promise.new_with_change_func(on_offer_created, webrtcbin, None)
    webrtcbin.emit("create-offer", None, promise)

def on_offer_created(promise, webrtcbin, _):
    """Handles SDP offer creation."""
    print("Offer created, setting local description...")
    promise.wait()
    reply = promise.get_reply()
    offer = reply['offer']

    # Set the local description
    promise = Gst.Promise.new()
    webrtcbin.emit('set-local-description', offer, promise)
    promise.interrupt()

    # Send the SDP offer to the remote peer
    asyncio.get_event_loop().create_task(send_sdp_offer(offer))

async def send_sdp_offer(offer):
    """Send the SDP offer to the remote peer over WebSocket."""
    if not websocket:
        print("No WebSocket connection, cannot send offer.")
        return
    
    text = offer.sdp.as_text()
    print ('Sending offer:\n%s' % text)
    msg = json.dumps({'sdp': {'type': 'offer', 'sdp': text}})
    try:
        await websocket.send(msg)
    except Exception as e:
        print(f"Failed to send SDP offer: {e}")

def start_pipeline():
    """Sets up and starts the GStreamer pipeline with WebRTC."""
    # Create the GStreamer pipeline
    pipeline = Gst.parse_launch('webrtcbin name=sendrecv bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 videotestsrc is-live=true pattern=ball ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv.')

    # Get the webrtcbin element
    webrtcbin = pipeline.get_by_name('sendrecv')

    # Connect to signals
    webrtcbin.connect('on-negotiation-needed', on_negotiation_needed)
    # webrtcbin.connect('on-ice-candidate', on_ice_candidate)
    # webrtcbin.connect('pad-added', on_incoming_stream)

    # Start the pipeline (this is where the negotiation would kick off)
    pipeline.set_state(Gst.State.PLAYING)

async def main():
    """Main entry point for the application."""
    Gst.init(None)

    await connect_to_server()
    start_pipeline()

    # Run the main loop to keep it active and handle signals
    loop = GLib.MainLoop()
    loop.run()

if __name__ == "__main__":
    asyncio.run(main())
