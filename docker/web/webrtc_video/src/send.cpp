#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/gstsdp.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <X11/Xlib.h>

class WebRTCSend {
public:
    WebRTCSend() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        this->pipeline = nullptr;
        this->webrtcbin = nullptr;
    }

    ~WebRTCSend() {
        ws_running = false;
        ws_cv.notify_all();
        if (ws_thread.joinable()) {
            ws_thread.join();
        }
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
        if (webrtcbin) {
            gst_object_unref(webrtcbin);
        }
    }

    void connect() {
        try {
            // Set up WebSocket connection using websocketpp
            websocketpp::client<websocketpp::config::asio_client> client;
            client.init_asio();

            client.set_open_handler(std::bind(&WebRTCSend::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCSend::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection("ws://0.0.0.0:8765", ec);

            if (ec) {
                std::cout << "Connection error: " << ec.message() << std::endl;
            }

            client.connect(con);
            client.run();

        } catch (const std::exception& e) {
            std::cout << "Failed to connect: " << e.what() << std::endl;
        }
    }

    void on_open(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>* c) {
        std::cout << "Connected! Waiting for HELLO message..." << std::endl;

        global_hdl = hdl;
        global_client = c;

        //ws_thread = std::thread(&WebRTCSend::process_ws_queue, this);
    }

    void on_msg(websocketpp::connection_hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg) {
        std::string payload = msg->get_payload();

        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(payload);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            if (!jsonMsg.isObject()) {
                std::cout << "Error: JSON message is not an object" << std::endl;
                return;
            }

            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "HELLO") {
                std::cout << "Received HELLO." << std::endl;
                Json::Value msg;
                msg["status"] = "OK";
                Json::StreamWriterBuilder writer;
                std::string message = Json::writeString(writer, msg);
                send_ws(message);
                setup_pipeline();
            } else if (jsonMsg.isMember("ice")) {
                std::cout << "Received ICE candidate." << std::endl;
                handle_ice(payload);
            } else if (jsonMsg.isMember("sdp")) {
                std::cout << "Received SDP answer." << std::endl;
                handle_sdp(payload);
            } else {
                std::cout << "Unknown JSON message type" << std::endl;
            }
        } else {
            std::cout << "Failed to parse JSON: " << errs << std::endl;
        }
    }

    // void process_ws_queue() {
    //     while (ws_running) {
    //         std::unique_lock<std::mutex> lock(ws_mutex);
    //         ws_cv.wait(lock, [this] { return !msg_queue.empty() || !ws_running; });

    //         std::string msg = msg_queue.front();
    //         msg_queue.pop();
    //         lock.unlock();

    //         websocketpp::lib::error_code ec;
    //         global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
    //         if (ec) {
    //             std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
    //         } else {
    //             std::cout << "Sent message over WebSocket: " << msg << std::endl;
    //         }
    //     }
    // }

    void send_ws(const std::string& msg) {
        websocketpp::lib::error_code ec;
        global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
        if (ec) {
            std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
        } else {
            std::cout << "Sent message over WebSocket: " << msg << std::endl;
        }
    }

    // void queue_ws(const std::string& msg) {
    //     std::lock_guard<std::mutex> lock(ws_mutex);
    //     msg_queue.push(msg);
    //     ws_cv.notify_one();
    // }

    void setup_pipeline() {
        // Create GStreamer pipeline
        GError* error = nullptr;
        pipeline = gst_parse_launch("webrtcbin name=sendrecv bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 videotestsrc is-live=true ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv. \
            videotestsrc is-live=true ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv. \
            videotestsrc is-live=true ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! sendrecv.", &error);
        if (error) {
            std::cerr << "ERROR: Could not create GStreamer pipeline: " << error->message << std::endl;
            g_error_free(error);
            return;
        }

        if (!pipeline) {
            std::cerr << "ERROR: Could not create GStreamer pipeline." << std::endl;
            return;
        }

        // Get webrtcbin element
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendrecv");

        if (!webrtcbin) {
            std::cerr << "ERROR: Could not get WebRTC element." << std::endl;
            return;
        }

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "latency", 0, "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(&WebRTCSend::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCSend::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCSend::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    static void send_ice_candidate(GstElement* webrtcbin, guint mlineindex, gchar* candidate, gpointer user_data) {
        std::cout << "Sending ICE candidate: " << candidate << std::endl;

        // Create a JSON message to send over WebSocket
        Json::Value icemsg;
        icemsg["ice"]["candidate"] = candidate;
        icemsg["ice"]["sdpMLineIndex"] = static_cast<int>(mlineindex);  // Casting mlineindex to int for JSON

        // Convert JSON message to string
        Json::StreamWriterBuilder writer;
        std::string icemsg_str = Json::writeString(writer, icemsg);

        // Send the ICE candidate over WebSocket
        static_cast<WebRTCSend*>(user_data)->send_ws(icemsg_str);
    }

    void handle_ice(const std::string& ice) {
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(ice);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            std::string candidate = jsonMsg["ice"]["candidate"].asString();
            int sdpMLineIndex = jsonMsg["ice"]["sdpMLineIndex"].asInt();

            if (!webrtcbin) {
                std::cerr << "ERROR: Could not get WebRTC element." << std::endl;
                return;
            }

            g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
        } else {
            std::cerr << "Failed to parse ICE candidate: " << errs << std::endl;
        }
    }

    static void on_negotiation_needed(GstElement* webrtcbin, gpointer user_data) {
        std::cout << "Negotiation needed" << std::endl;

        GstPromise* promise = gst_promise_new_with_change_func(on_offer_created, user_data, nullptr);
        g_signal_emit_by_name(webrtcbin, "create-offer", nullptr, promise);
    }

    static void on_offer_created(GstPromise* promise, gpointer user_data) {
        GstWebRTCSessionDescription* offer = nullptr;
        const GstStructure* reply = gst_promise_get_reply(promise);
        gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
        gst_promise_unref(promise);

        if (!offer) {
            std::cerr << "Failed to create offer" << std::endl;
            return;
        }

        gchar* sdp_str = gst_sdp_message_as_text(offer->sdp);
        std::cout << "Created offer: " << sdp_str << std::endl;

        Json::Value sdpmsg;
        sdpmsg["sdp"]["type"] = "offer";
        sdpmsg["sdp"]["sdp"] = sdp_str;

        Json::StreamWriterBuilder writer;
        std::string sdpmsg_str = Json::writeString(writer, sdpmsg);

        static_cast<WebRTCSend*>(user_data)->send_ws(sdpmsg_str);

        g_signal_emit_by_name(static_cast<WebRTCSend*>(user_data)->webrtcbin, "set-local-description", offer, nullptr);

        g_free(sdp_str);
        gst_webrtc_session_description_free(offer);
    }

    void handle_sdp(const std::string& sdp) {
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(sdp);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            std::string sdp_str = jsonMsg["sdp"]["sdp"].asString();
            GstSDPMessage* sdp_msg = nullptr;
            gst_sdp_message_new(&sdp_msg);
            GstSDPResult result = gst_sdp_message_parse_buffer((guint8*)sdp_str.c_str(), sdp_str.size(), sdp_msg);

            if (result != GST_SDP_OK) {
                std::cerr << "Failed to parse SDP message: " << result << std::endl;
                return;
            }

            GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
            if (!answer) {
                std::cerr << "Failed to create WebRTC session description" << std::endl;
                gst_sdp_message_free(sdp_msg);
                return;
            }

            g_signal_emit_by_name(webrtcbin, "set-remote-description", answer, nullptr);
            gst_webrtc_session_description_free(answer);
        } else {
            std::cerr << "Failed to parse SDP answer: " << errs << std::endl;
        }
    }

    static void on_incoming_stream(GstElement* webrtcbin, GstPad* pad, WebRTCSend* self) {
        std::cout << "Received incoming stream." << std::endl;

        GstPad* sinkpad;
        GstElement* decodebin = gst_element_factory_make("decodebin", nullptr);
        gst_bin_add(GST_BIN(self->pipeline), decodebin);
        gst_element_sync_state_with_parent(decodebin);

        g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), self);

        sinkpad = gst_element_get_static_pad(decodebin, "sink");
        gst_pad_link(pad, sinkpad);
        gst_object_unref(sinkpad);
    }

    static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, WebRTCSend* self) {
        GstCaps* caps = gst_pad_get_current_caps(pad);
        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);

        GstElement* conv = nullptr;
        GstElement* sink = nullptr;

        if (g_str_has_prefix(name, "video")) {
            conv = gst_element_factory_make("videoconvert", nullptr);
            sink = gst_element_factory_make("autovideosink", nullptr);
        } else if (g_str_has_prefix(name, "audio")) {
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        }

        if (conv && sink) {
            gst_bin_add_many(GST_BIN(self->pipeline), conv, sink, nullptr);
            gst_element_sync_state_with_parent(conv);
            gst_element_sync_state_with_parent(sink);

            GstPad* sinkpad = gst_element_get_static_pad(conv, "sink");
            gst_pad_link(pad, sinkpad);
            gst_object_unref(sinkpad);

            GstPad* srcpad = gst_element_get_static_pad(conv, "src");
            GstPad* sinkpad2 = gst_element_get_static_pad(sink, "sink");
            gst_pad_link(srcpad, sinkpad2);
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad2);
        }

        gst_caps_unref(caps);
    }

private:
    GstElement* pipeline;
    GstElement* webrtcbin;
    websocketpp::connection_hdl global_hdl;
    websocketpp::client<websocketpp::config::asio_client>* global_client;

    std::queue<std::string> msg_queue;
    std::mutex ws_mutex;
    std::condition_variable ws_cv;
    std::thread ws_thread;
    std::atomic<bool> ws_running{true};
};

int main() {
    // Call XInitThreads() to enable thread safety in X11
    if (!XInitThreads()) {
        std::cerr << "Failed to initialize X11 threading!" << std::endl;
        return 1;
    }

    WebRTCSend recv;
    recv.connect();

    return 0;
}