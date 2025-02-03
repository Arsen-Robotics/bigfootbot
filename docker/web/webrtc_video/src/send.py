#!/usr/bin/env python3

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GLib
import websockets

SIGNALING_SERVER = 'ws://localhost:8765'
websocket = None

# Initialize GStreamer
Gst.init(None)

def connect_to_server():
    global websocket
    websocket = websockets.connect(SIGNALING_SERVER)
    print("Connected to signaling server")

def on_negotiation_needed(webrtcbin):
    print("Negotiation needed, creating offer...")
    promise = Gst.Promise.new_with_change_func(on_offer_created, webrtcbin, None)
    webrtcbin.emit("create-offer", None, promise)

def on_offer_created(promise, webrtcbin, _):
    print("Offer created, setting local description...")
    promise.wait()
    reply = promise.get_reply()
    offer = reply['offer']
    promise = Gst.Promise.new()
    webrtcbin.emit('set-local-description', offer, promise)
    promise.interrupt()
    send_sdp_offer(offer)

def send_sdp_offer(offer):
    text = offer.sdp.as_text()
    print ('Sending offer:\n%s' % text)
    # msg = json.dumps({'sdp': {'type': 'offer', 'sdp': text}})
    # loop = asyncio.new_event_loop()
    # loop.run_until_complete(self.conn.send(msg))
    # loop.close()

def start_pipeline():
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

def main():
    connect_to_server()
    start_pipeline()

    # Run the main loop to keep it active and handle signals
    loop = GLib.MainLoop()
    loop.run()

if __name__ == "__main__":
    main()
