// Required headers
#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/gstsdp.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <glib.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

/**
 * @brief Main class handling WebRTC video streaming for Raspberry Pi
 * 
 * This class manages the WebRTC connection, GStreamer pipeline,
 * and WebSocket signaling to stream video from a Raspberry Pi camera
 * to a browser or other WebRTC client
 */
class WebRTCSend {
public:
    /**
     * @brief Constructor - initializes GStreamer and member variables
     */
    WebRTCSend() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        this->pipeline = nullptr;
        this->webrtcbin = nullptr;
        ws_running = true; // this is a flag to control the WebSocket thread
    }

    /**
     * @brief Destructor - cleans up resources
     */
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

    /**
     * @brief Establishes WebSocket connection to signaling server
     * 
     * @param server_address WebSocket server address (default: localhost)
     * @param port WebSocket server port (default: 8765)
     */
    void connect(const std::string& server_address = "0.0.0.0", int port = 8765) {
        try {
            // Set up WebSocket connection using websocketpp
            websocketpp::client<websocketpp::config::asio_client> client;
            client.init_asio();

            client.set_open_handler(std::bind(&WebRTCSend::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCSend::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            std::string uri = "ws://" + server_address + ":" + std::to_string(port);
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection(uri, ec);

            if (ec) {
                std::cout << "Connection error: " << ec.message() << std::endl;
            }

            client.connect(con);
            client.run();

        } catch (const std::exception& e) {
            std::cout << "Failed to connect: " << e.what() << std::endl;
        }
    }

    /**
     * @brief Handler called when WebSocket connection is established
     * 
     * @param hdl Connection handle
     * @param c Pointer to WebSocket client
     */
    void on_open(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>* c) {
        std::cout << "Connected! Waiting for HELLO message..." << std::endl;

        global_hdl = hdl;
        global_client = c;

        ws_thread = std::thread(&WebRTCSend::process_ws_queue, this);
    }

    /**
     * @brief Handler for incoming WebSocket messages
     * 
     * Processes different types of messages including:
     * - HELLO handshake
     * - ICE candidates
     * - SDP offers/answers
     */
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
                queue_ws(message);
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

    /**
     * @brief Processes queued WebSocket messages
     * 
     * Runs in a separate thread to handle outgoing WebSocket messages
     */
    void process_ws_queue() {
        while (ws_running) {
            std::unique_lock<std::mutex> lock(ws_mutex);
            ws_cv.wait(lock, [this] { return !msg_queue.empty() || !ws_running; });

            if (!ws_running) break;
            
            if (!msg_queue.empty()) {
                std::string msg = msg_queue.front();
                msg_queue.pop();
                lock.unlock();

                websocketpp::lib::error_code ec;
                global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
                if (ec) {
                    std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
                } else {
                    std::cout << "Sent message over WebSocket: " << msg << std::endl;
                }
            }
        }
    }

    /**
     * @brief Queues a message to be sent over WebSocket
     * 
     * @param msg Message to be queued
     */
    void queue_ws(const std::string& msg) {
        std::lock_guard<std::mutex> lock(ws_mutex);
        msg_queue.push(msg);
        ws_cv.notify_one();
    }

    /**
     * @brief Sets up the GStreamer pipeline for video streaming
     * 
     * Creates and configures a pipeline with:
     * - v4l2src source for Raspberry Pi camera
     * - Hardware H.264 encoding using v4l2h264enc
     * - WebRTC transmission
     */
    void setup_pipeline() {
        // Create GStreamer pipeline - optimized for Raspberry Pi 4
        GError* error = nullptr;
        
        // Pipeline uses:
        // 1. v4l2src: Standard Video4Linux source (for USB or RPi camera)
        // 2. videoconvert: Converts between video formats
        // 3. v4l2h264enc: RPi4 hardware H.264 encoder
        // 4. h264parse: Parses H.264 streams
        // 5. rtph264pay: Packetizes H.264 for RTP
        pipeline = gst_parse_launch("webrtcbin name=sendrecv bundle-policy=max-bundle latency=0 \
            stun-server=stun://stun.l.google.com:19302 \
            v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! videoconvert ! v4l2h264enc extra-controls=\"controls,h264_profile=4,video_bitrate=1000000\" \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.", &error);
            
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
        g_object_set(G_OBJECT(webrtcbin), "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(&WebRTCSend::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCSend::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCSend::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        std::cout << "Pipeline started. Streaming from camera..." << std::endl;
    }

    /**
     * @brief Callback for sending ICE candidates
     * 
     * Called when a new ICE candidate is discovered
     */
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
        static_cast<WebRTCSend*>(user_data)->queue_ws(icemsg_str);
    }

    /**
     * @brief Handles incoming ICE candidates
     * 
     * @param ice JSON string containing ICE candidate information
     */
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

    /**
     * @brief Callback when WebRTC negotiation is needed
     */
    static void on_negotiation_needed(GstElement* webrtcbin, gpointer user_data) {
        std::cout << "Negotiation needed" << std::endl;

        GstPromise* promise = gst_promise_new_with_change_func(on_offer_created, user_data, nullptr);
        g_signal_emit_by_name(webrtcbin, "create-offer", nullptr, promise);
    }

    /**
     * @brief Callback when SDP offer is created
     */
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

        static_cast<WebRTCSend*>(user_data)->queue_ws(sdpmsg_str);

        g_signal_emit_by_name(static_cast<WebRTCSend*>(user_data)->webrtcbin, "set-local-description", offer, nullptr);

        g_free(sdp_str);
        gst_webrtc_session_description_free(offer);
    }

    /**
     * @brief Handles incoming SDP messages
     * 
     * @param sdp JSON string containing SDP information
     */
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

    /**
     * @brief Callback for handling incoming media streams
     */
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

    /**
     * @brief Callback when decodebin creates a new pad
     * 
     * Sets up appropriate elements for handling decoded audio/video streams
     */
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
    GstElement* pipeline;                // Main GStreamer pipeline
    GstElement* webrtcbin;              // WebRTC element
    websocketpp::connection_hdl global_hdl;  // WebSocket connection handle
    websocketpp::client<websocketpp::config::asio_client>* global_client;  // WebSocket client

    std::queue<std::string> msg_queue;   // Queue for outgoing WebSocket messages
    std::mutex ws_mutex;                 // Mutex for thread safety
    std::condition_variable ws_cv;       // Condition variable for message queue
    std::thread ws_thread;               // Thread for processing WebSocket messages
    std::atomic<bool> ws_running{true};  // Flag to control WebSocket thread
};

/**
 * @brief Main entry point
 * 
 * Creates WebRTCSend instance and runs the main loop
 */
int main(int argc, char* argv[]) {
    std::string server_address = "0.0.0.0";  // Default to localhost
    int server_port = 8765;                 // Default port
    
    // Parse command line arguments for server address and port
    if (argc > 1) {
        server_address = argv[1];
    }
    if (argc > 2) {
        server_port = std::stoi(argv[2]);
    }
    
    std::cout << "Starting WebRTC sender for Raspberry Pi" << std::endl;
    std::cout << "Connecting to signaling server at: ws://" << server_address << ":" << server_port << std::endl;

    WebRTCSend sender;
    std::thread t([&sender, server_address, server_port]() { 
        sender.connect(server_address, server_port); 
    });

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    t.join();

    return 0;
} 