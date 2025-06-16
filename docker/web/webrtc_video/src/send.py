import asyncio
import json
import websockets
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp

class WebRTCSend:
    def __init__(self):
        self.SIGNALING_SERVER = 'ws://0.0.0.0:8765'
        self.websocket = None
        self.webrtcbin = None
        self.pipeline = None

    async def connect(self):
        """Establish WebSocket connection to the signaling server."""
        try:
            self.websocket = await websockets.connect(self.SIGNALING_SERVER)
            print("Connected to signaling server")
        except Exception as e:
            print(f"Failed to connect to signaling server: {e}")

        #await self.websocket.send(json.dumps('HELLO'))

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

        # Because of older GStreamer version on NVIDIA, using second variant
        #offer = reply['offer']
        offer = reply.get_value('offer')

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

    def start_pipeline(self):
        """Sets up and starts the GStreamer pipeline with WebRTC."""
        # Create the GStreamer pipeline
        self.pipeline = Gst.parse_launch('webrtcbin name=sendrecv bundle-policy=max-bundle latency=0 \
            stun-server=stun://stun.l.google.com:19302 \
            v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! nvv4l2h264enc bitrate=2300000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            \
            v4l2src device=/dev/cam-microdia ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! nvv4l2h264enc bitrate=2300000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            \
            nvarguscamerasrc sensor-mode=4 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! nvv4l2h264enc bitrate=2300000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            \
            v4l2src device=/dev/video20 ! video/x-raw,width=640,height=360,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! nvv4l2h264enc bitrate=2300000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.')

        # Working pipeline for CPU encoding (x264enc)

        # self.pipeline = Gst.parse_launch('webrtcbin name=sendrecv bundle-policy=max-bundle latency=0 \
        #     stun-server=stun://stun.l.google.com:19302 \
        #     v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 \
        #     ! videoconvert ! video/x-raw,format=I420 ! queue max-size-buffers=1 max-size-time=20000000 max-size-bytes=0 leaky=downstream \
        #     ! x264enc tune=zerolatency speed-preset=ultrafast rc-lookahead=0 bitrate=1000 key-int-max=30 qp-min=18 qp-max=25 \
        #     ! h264parse ! rtph264pay config-interval=1 pt=96 \
        #     ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.')
        
        # self.pipeline = Gst.parse_launch('webrtcbin name=sendrecv bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 videotestsrc is-live=true ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv. \
        # videotestsrc is-live=true ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv. \
        # audiotestsrc is-live=true ! audioconvert ! audioresample ! opusenc ! rtpopuspay ! application/x-rtp,media=audio,encoding-name=OPUS,payload=96 ! sendrecv.')
        # Get the webrtcbin element
        
        self.webrtcbin = self.pipeline.get_by_name('sendrecv')

        # Can't use latency property on NVIDIA because of old GStreamer version
        #self.webrtcbin.set_property("latency", 0)
        self.webrtcbin.set_property("bundle-policy", "max-bundle")
        self.webrtcbin.set_property("stun-server", "stun://stun.l.google.com:19302")


        # Connect to signals
        self.webrtcbin.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtcbin.connect('on-ice-candidate', self.send_ice_candidate)
        self.webrtcbin.connect('pad-added', self.on_incoming_stream)

        # Start the pipeline (this is where the negotiation would kick off)
        self.pipeline.set_state(Gst.State.PLAYING)

    def handle_sdp(self, msg):
        """Handle incoming SDP answer."""
        sdp = msg['sdp']
        assert(sdp['type'] == 'answer')
        sdp = sdp['sdp']
        print ('Received answer:\n%s' % sdp)

        # Process the answer into a WebRTCSessionDescription
        res, sdpmsg = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(bytes(sdp.encode()), sdpmsg)
        answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)

        # Set the remote description (the answer)
        promise = Gst.Promise.new()
        self.webrtcbin.emit('set-remote-description', answer, promise)
        promise.interrupt()
    
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

        decodebin = Gst.ElementFactory.make('decodebin')
        decodebin.connect('pad-added', self.on_incoming_decodebin_stream)
        self.pipeline.add(decodebin)
        decodebin.sync_state_with_parent()
        self.webrtcbin.link(decodebin)

    def on_incoming_decodebin_stream(self, _, pad):
        """Handle incoming decodebin stream."""
        if not pad.has_current_caps():
            print(pad, 'has no caps, ignoring')
            return

        caps = pad.get_current_caps()
        assert len(caps)
        s = caps[0]
        name = s.get_name()

        if name.startswith('video'):
            pass
            # q = Gst.ElementFactory.make('queue')
            # conv = Gst.ElementFactory.make('videoconvert')
            # sink = Gst.ElementFactory.make('autovideosink')
            
            # # Minimize latency in queue (reduce buffering)
            # q.set_property("max-size-buffers", 1)
            # q.set_property("max-size-time", 0)
            # q.set_property("max-size-bytes", 0)
            # q.set_property("leaky", "downstream")  # Allow data to drop if too much buffering happens
            
            # # Disable sync on autovideosink for lower latency
            # sink.set_property("sync", False)

            # self.pipeline.add(q, conv, sink)
            # self.pipeline.sync_children_states()
            # pad.link(q.get_static_pad('sink'))
            # q.link(conv)
            # conv.link(sink)

        elif name.startswith('audio'):
            pass
            # q = Gst.ElementFactory.make('queue')
            # conv = Gst.ElementFactory.make('audioconvert')
            # resample = Gst.ElementFactory.make('audioresample')
            # sink = Gst.ElementFactory.make('autoaudiosink')
            
            # # Same low-latency settings for audio queue
            # q.set_property("max-size-buffers", 1)
            # q.set_property("max-size-time", 0)
            # q.set_property("max-size-bytes", 0)
            # q.set_property("leaky", "downstream")

            # self.pipeline.add(q, conv, resample, sink)
            # self.pipeline.sync_children_states()
            # pad.link(q.get_static_pad('sink'))
            # q.link(conv)
            # conv.link(resample)
            # resample.link(sink)
    
    async def listen(self):
        """Main loop to handle incoming messages."""
        async for message in self.websocket:
            msg = json.loads(message)
            if msg == {"status": "HELLO"}:
                print("Received HELLO from RECV peer")
                await self.websocket.send(json.dumps({'status': 'OK'}))
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
    webrtc_send = WebRTCSend()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(webrtc_send.connect())
    loop.run_until_complete(webrtc_send.listen())
