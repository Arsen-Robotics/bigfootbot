import asyncio
import json
import websockets
import gi
import ctypes

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

class WebRTCRecv:
    def __init__(self):
        self.SIGNALING_SERVER = 'ws://87.119.173.184:8765'
        self.websocket = None
        self.webrtcbin = None
        self.pipeline = None

        # Initialize X11 threading
        libX11 = ctypes.cdll.LoadLibrary("libX11.so")
        libX11.XInitThreads()

        self.pipeline = None
        self.compositor = None
        
        # Call setup_pipeline here to create and add the compositor
        self.setup_pipeline()

    def setup_pipeline(self):
        """Sets up the GStreamer pipeline."""
        # Create the pipeline
        self.pipeline = Gst.Pipeline.new("pipeline")

        # Create compositor to merge video streams
        self.compositor = Gst.ElementFactory.make("compositor", "video_mixer")

        if not self.compositor:
            print("ERROR: Could not create compositor.")
            return

        # Add compositor to pipeline
        self.pipeline.add(self.compositor)

        # Set the compositor to playing state
        self.compositor.set_state(Gst.State.PLAYING)

    async def connect(self):
        """Establish WebSocket connection to the signaling server."""
        try:
            self.websocket = await websockets.connect(self.SIGNALING_SERVER)
            print("Connected to signaling server")
        except Exception as e:
            print(f"Failed to connect to signaling server: {e}")

        await self.websocket.send(json.dumps({"status": "HELLO"}))

    def on_negotiation_needed(self, _):
        """Triggered when WebRTC negotiation is needed."""
        print("Negotiation needed, creating offer...")
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, self.webrtcbin, None)
        self.webrtcbin.emit("create-offer", None, promise)

    def on_offer_created(self, promise, _, __):
        """Handles SDP offer creation."""
        print("Offer created, setting local description...")
        promise.wait()
        reply = promise.get_reply()
        offer = reply['offer']

        # Set the local description
        promise = Gst.Promise.new()
        self.webrtcbin.emit('set-local-description', offer, promise)
        promise.interrupt()

        # Send the SDP offer to the remote peer
        self.send_sdp_offer(offer)

    def send_sdp_offer(self, offer):
        """Send the SDP offer to the remote peer over WebSocket."""
        if not self.websocket:
            print("No WebSocket connection, cannot send offer.")
            return
        
        text = offer.sdp.as_text()
        print ('Sending offer:\n%s' % text)
        msg = json.dumps({'sdp': {'type': 'offer', 'sdp': text}})
        try:
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self.websocket.send(msg))
            loop.close()
        except Exception as e:
            print(f"Failed to send SDP offer: {e}")

    def send_sdp_answer(self, answer):
        """Send the SDP answer to the remote peer over WebSocket."""
        if not self.websocket:
            print("No WebSocket connection, cannot send answer.")
            return
        
        text = answer.sdp.as_text()
        print ('Sending answer:\n%s' % text)
        msg = json.dumps({'sdp': {'type': 'answer', 'sdp': text}})
        try:
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self.websocket.send(msg))
            loop.close()
        except Exception as e:
            print(f"Failed to send SDP answer: {e}")

    def start_pipeline(self):
        """Sets up and starts the GStreamer pipeline with WebRTC."""
        # Create the GStreamer pipeline
        self.pipeline = Gst.parse_launch('webrtcbin name=recvonly bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 audiotestsrc ! audioconvert ! fakesink')

        # Get the webrtcbin element
        self.webrtcbin = self.pipeline.get_by_name('recvonly')

        self.webrtcbin.set_property("latency", 0)
        self.webrtcbin.set_property("bundle-policy", "max-bundle")
        self.webrtcbin.set_property("stun-server", "stun://stun.l.google.com:19302")


        # Connect to signals
        #self.webrtcbin.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtcbin.connect('on-ice-candidate', self.send_ice_candidate)
        self.webrtcbin.connect('pad-added', self.on_incoming_stream)

        # Start the pipeline (this is where the negotiation would kick off)
        self.pipeline.set_state(Gst.State.PLAYING)

    def handle_sdp(self, msg):
        """Handle incoming SDP offer and create SDP answer."""
        sdp = msg['sdp']
        assert(sdp['type'] == 'offer')
        sdp = sdp['sdp']
        print ('Received offer:\n%s' % sdp)

        # Process the offer into a WebRTCSessionDescription
        res, sdpmsg = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(bytes(sdp.encode()), sdpmsg)
        offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdpmsg)

        # Set the remote description
        promise = Gst.Promise.new()
        self.webrtcbin.emit('set-remote-description', offer, promise)
        promise.interrupt()

        # Create an SDP answer
        promise = Gst.Promise.new_with_change_func(self.on_answer_created, self.webrtcbin, None)
        self.webrtcbin.emit("create-answer", None, promise)

    def on_answer_created(self, promise, _, __):
        """Handles SDP answer creation."""
        print("Answer created, setting local description...")
        promise.wait()
        reply = promise.get_reply()
        answer = reply['answer']

        # Set the local description
        promise = Gst.Promise.new()
        self.webrtcbin.emit('set-local-description', answer, promise)
        promise.interrupt()

        # Send the SDP answer to the remote peer
        self.send_sdp_answer(answer)

    def handle_ice(self, msg):
        ice = msg['ice']
        candidate = ice['candidate']
        sdpmlineindex = ice['sdpMLineIndex']
        self.webrtcbin.emit('add-ice-candidate', sdpmlineindex, candidate)

    def send_ice_candidate(self, _, mlineindex, candidate):
        """Send ICE candidate to the remote peer over WebSocket."""
        print(f"Sending ICE candidate: {candidate}")
        icemsg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self.websocket.send(icemsg))
        loop.close()

    def on_incoming_stream(self, _, pad):
        """Handle incoming stream from the remote peer."""
        if pad.direction != Gst.PadDirection.SRC:
            return
        
        asyncio.run(self.process_stream(pad))

    async def process_stream(self, pad):
        decodebin = Gst.ElementFactory.make('decodebin')
        decodebin.connect('pad-added', self.on_incoming_decodebin_stream)
        self.pipeline.add(decodebin)
        decodebin.sync_state_with_parent()
        self.webrtcbin.link(decodebin)

    def on_incoming_decodebin_stream(self, _, pad):
        """Handle incoming decodebin stream."""
        # Ensure caps are available before continuing
        caps = pad.get_current_caps()
        if not caps:
            print("Pad has no caps, ignoring")
            return

        # Process the pad asynchronously
        asyncio.run(self.process_pad(pad))

    async def process_pad(self, pad):
        caps = pad.get_current_caps()
        assert len(caps)
        s = caps[0]
        name = s.get_name()

        if name.startswith('video'):
            q = Gst.ElementFactory.make('queue')
            conv = Gst.ElementFactory.make('videoconvert')            
            sink = Gst.ElementFactory.make('xvimagesink')
            
            # Minimize latency in queue (reduce buffering)
            q.set_property("max-size-buffers", 1)
            q.set_property("max-size-time", 0)
            q.set_property("max-size-bytes", 0)
            q.set_property("leaky", "downstream")  # Allow data to drop if too much buffering happens
            
            # Disable sync on autovideosink for lower latency
            sink.set_property("sync", False)

            self.pipeline.add(q, conv, sink)
            self.pipeline.sync_children_states()
            pad.link(q.get_static_pad('sink'))
            q.link(conv)
            conv.link(sink)

        elif name.startswith('audio'):
            q = Gst.ElementFactory.make('queue')
            conv = Gst.ElementFactory.make('audioconvert')
            resample = Gst.ElementFactory.make('audioresample')
            sink = Gst.ElementFactory.make('autoaudiosink')
            
            # Same low-latency settings for audio queue
            q.set_property("max-size-buffers", 1)
            q.set_property("max-size-time", 0)
            q.set_property("max-size-bytes", 0)
            q.set_property("leaky", "downstream")

            self.pipeline.add(q, conv, resample, sink)
            self.pipeline.sync_children_states()
            pad.link(q.get_static_pad('sink'))
            q.link(conv)
            conv.link(resample)
            resample.link(sink)

    async def listen(self):
        """Main loop to handle incoming messages."""
        async for message in self.websocket:
            msg = json.loads(message)
            if msg == {"status": "OK"}:
                print("Received OK from SEND peer")
                self.start_pipeline()
            elif 'sdp' in msg:
                self.handle_sdp(msg)
            elif 'ice' in msg:
                self.handle_ice(msg)
            elif msg.startswith('ERROR'):
                print(msg)
                return 1
        return 0

if __name__ == "__main__":
    Gst.init(None)
    webrtc_recv = WebRTCRecv()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(webrtc_recv.connect())
    loop.run_until_complete(webrtc_recv.listen())

    # Start the main event loop
    #GLib.MainLoop().run()