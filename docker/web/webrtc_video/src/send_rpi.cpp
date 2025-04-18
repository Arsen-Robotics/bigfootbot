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
#include <functional>
#include <memory>
#include <csignal>

// Define log macros to improve readability
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl

// Define WebSocket client type
using WebSocketClient = websocketpp::client<websocketpp::config::asio_client>;
using ConnectionHandle = websocketpp::connection_hdl;

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
    WebRTCSend() : pipeline(nullptr), webrtcbin(nullptr), ws_running(true) {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        LOG_INFO("GStreamer initialized");
    }

    /**
     * @brief Destructor - cleans up resources
     */
    ~WebRTCSend() {
        cleanup();
    }
    
    /**
     * @brief Clean up all resources
     */
    void cleanup() {
        LOG_INFO("Cleaning up resources");
        
        // Stop WebSocket message processing thread
        ws_running = false;
        ws_cv.notify_all();
        if (ws_thread.joinable()) {
            ws_thread.join();
        }
        
        // Clean up GStreamer resources
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        
        if (webrtcbin) {
            gst_object_unref(webrtcbin);
            webrtcbin = nullptr;
        }
        
        LOG_INFO("Cleanup complete");
    }

    /**
     * @brief Establishes WebSocket connection to signaling server
     * 
     * @param server_address WebSocket server address (default: localhost)
     * @param port WebSocket server port (default: 8765)
     */
    void connect(const std::string& server_address = "0.0.0.0", int port = 8765) {
        try {
            LOG_INFO("Connecting to signaling server at ws://" + server_address + ":" + std::to_string(port));
            
            // Initialize WebSocket client
            client.init_asio();
            client.clear_access_channels(websocketpp::log::alevel::all);
            client.set_error_channels(websocketpp::log::elevel::warn);
            
            // Set up event handlers
            client.set_open_handler(std::bind(&WebRTCSend::on_open, this, std::placeholders::_1));
            client.set_message_handler(std::bind(&WebRTCSend::on_msg, this, 
                                                 std::placeholders::_1, std::placeholders::_2));
            client.set_close_handler(std::bind(&WebRTCSend::on_close, this, std::placeholders::_1));
            
            // Connect to the signaling server
            websocketpp::lib::error_code ec;
            std::string uri = "ws://" + server_address + ":" + std::to_string(port);
            WebSocketClient::connection_ptr conn = client.get_connection(uri, ec);

            if (ec) {
                LOG_ERROR("Connection error: " + ec.message());
                return;
            }

            client.connect(conn);
            client.run();

        } catch (const std::exception& e) {
            LOG_ERROR("Failed to connect: " + std::string(e.what()));
        }
    }

    /**
     * @brief Handler called when WebSocket connection is established
     * 
     * @param hdl Connection handle
     */
    void on_open(ConnectionHandle hdl) {
        LOG_INFO("Connected to signaling server! Waiting for HELLO message...");

        global_hdl = hdl;
        
        // Start the message processing thread
        ws_thread = std::thread(&WebRTCSend::process_ws_queue, this);
    }
    
    /**
     * @brief Handler called when WebSocket connection is closed
     * 
     * @param hdl Connection handle
     */
    void on_close(ConnectionHandle hdl) {
        LOG_INFO("Connection to signaling server closed");
        
        // Stop the pipeline when connection is closed
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
    }

    /**
     * @brief Handler for incoming WebSocket messages
     * 
     * Processes different types of messages including:
     * - HELLO handshake
     * - ICE candidates
     * - SDP offers/answers
     */
    void on_msg(ConnectionHandle, WebSocketClient::message_ptr msg) {
        try {
            std::string payload = msg->get_payload();
            Json::CharReaderBuilder reader;
            Json::Value jsonMsg;
            std::string errors;
            std::istringstream s(payload);

            if (!Json::parseFromStream(reader, s, &jsonMsg, &errors)) {
                LOG_ERROR("Failed to parse JSON: " + errors);
                return;
            }

            if (!jsonMsg.isObject()) {
                LOG_ERROR("Error: JSON message is not an object");
                return;
            }

            // Process different message types
            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "HELLO") {
                LOG_INFO("Received HELLO from signaling server");
                
                // Send acknowledgement
                Json::Value response;
                response["status"] = "OK";
                response["message"] = "Ready to stream";
                
                Json::StreamWriterBuilder writer;
                std::string message = Json::writeString(writer, response);
                queue_ws(message);
                
                // Set up the streaming pipeline
                setup_pipeline();
            } 
            else if (jsonMsg.isMember("ice")) {
                LOG_INFO("Received ICE candidate");
                handle_ice(payload);
            } 
            else if (jsonMsg.isMember("sdp")) {
                LOG_INFO("Received SDP answer");
                handle_sdp(payload);
            } 
            else {
                LOG_INFO("Received unknown message type");
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Error processing message: " + std::string(e.what()));
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
                client.send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
                
                if (ec) {
                    LOG_ERROR("Error sending WebSocket message: " + ec.message());
                } else {
                    LOG_DEBUG("Sent: " + msg);
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
     * - Hardware H.264 encoding using openh264enc
     * - WebRTC transmission
     */
    void setup_pipeline() {
        // Define pipeline configuration - optimized for Raspberry Pi with detailed encoder settings
        std::string pipeline_desc = 
            "webrtcbin name=sendrecv bundle-policy=max-bundle "
            "stun-server=stun://stun.l.google.com:19302 "
            "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=I420 ! "
            "queue max-size-buffers=1 max-size-time=20000000 max-size-bytes=0 leaky=downstream ! "
            "x264enc tune=zerolatency speed-preset=ultrafast rc-lookahead=0 bitrate=1000 "
            "key-int-max=30 qp-min=18 qp-max=25 ! h264parse ! rtph264pay config-interval=1 pt=96 ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.";
            
        LOG_INFO("Creating pipeline");
        
        // Parse the pipeline
        GError* error = nullptr;
        pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);
            
        if (error) {
            LOG_ERROR("Could not create GStreamer pipeline: " + std::string(error->message));
            g_error_free(error);
            return;
        }

        if (!pipeline) {
            LOG_ERROR("Could not create GStreamer pipeline");
            return;
        }

        // Get webrtcbin element from the pipeline
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendrecv");
        if (!webrtcbin) {
            LOG_ERROR("Could not find WebRTC element in the pipeline");
            return;
        }

        // Connect to signaling callbacks
        g_signal_connect(webrtcbin, "on-negotiation-needed", 
                         G_CALLBACK(&WebRTCSend::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", 
                         G_CALLBACK(&WebRTCSend::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", 
                         G_CALLBACK(&WebRTCSend::on_incoming_stream), this);

        // Start the pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            LOG_ERROR("Failed to start the pipeline");
            return;
        }
        
        LOG_INFO("Pipeline started successfully. Streaming from camera...");
    }

    /**
     * @brief Callback for sending ICE candidates
     * 
     * Called when a new ICE candidate is discovered
     */
    static void send_ice_candidate(GstElement* webrtcbin, guint mlineindex, 
                                  gchar* candidate, gpointer user_data) {
        LOG_DEBUG("Discovered ICE candidate: " + std::string(candidate));

        // Create a JSON message to send over WebSocket
        Json::Value icemsg;
        icemsg["ice"]["candidate"] = candidate;
        icemsg["ice"]["sdpMLineIndex"] = static_cast<int>(mlineindex);

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
                LOG_ERROR("Could not get WebRTC element");
                return;
            }

            LOG_DEBUG("Adding ICE candidate: " + candidate);
            g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
        } else {
            LOG_ERROR("Failed to parse ICE candidate: " + errs);
        }
    }

    /**
     * @brief Callback when WebRTC negotiation is needed
     */
    static void on_negotiation_needed(GstElement* webrtcbin, gpointer user_data) {
        LOG_INFO("WebRTC negotiation needed, creating offer");

        GstPromise* promise = gst_promise_new_with_change_func(on_offer_created, user_data, nullptr);
        g_signal_emit_by_name(webrtcbin, "create-offer", nullptr, promise);
    }

    /**
     * @brief Callback when SDP offer is created
     */
    static void on_offer_created(GstPromise* promise, gpointer user_data) {
        WebRTCSend* self = static_cast<WebRTCSend*>(user_data);
        GstWebRTCSessionDescription* offer = nullptr;
        const GstStructure* reply = gst_promise_get_reply(promise);
        gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
        gst_promise_unref(promise);

        if (!offer) {
            LOG_ERROR("Failed to create offer");
            return;
        }

        // Convert SDP to string
        gchar* sdp_str = gst_sdp_message_as_text(offer->sdp);
        LOG_DEBUG("Created offer SDP");

        // Create JSON message with the offer
        Json::Value sdpmsg;
        sdpmsg["sdp"]["type"] = "offer";
        sdpmsg["sdp"]["sdp"] = sdp_str;

        // Convert to string and send
        Json::StreamWriterBuilder writer;
        std::string sdpmsg_str = Json::writeString(writer, sdpmsg);
        self->queue_ws(sdpmsg_str);

        // Set as local description
        g_signal_emit_by_name(self->webrtcbin, "set-local-description", offer, nullptr);

        // Free resources
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
            
            // Parse the SDP message
            GstSDPMessage* sdp_msg = nullptr;
            gst_sdp_message_new(&sdp_msg);
            GstSDPResult result = gst_sdp_message_parse_buffer(
                (guint8*)sdp_str.c_str(), sdp_str.size(), sdp_msg);

            if (result != GST_SDP_OK) {
                LOG_ERROR("Failed to parse SDP message: " + std::to_string(result));
                return;
            }

            // Create answer session description
            GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(
                GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
                
            if (!answer) {
                LOG_ERROR("Failed to create WebRTC session description");
                gst_sdp_message_free(sdp_msg);
                return;
            }

            // Set as remote description
            LOG_INFO("Setting remote description (SDP answer)");
            g_signal_emit_by_name(webrtcbin, "set-remote-description", answer, nullptr);
            gst_webrtc_session_description_free(answer);
        } else {
            LOG_ERROR("Failed to parse SDP answer: " + errs);
        }
    }

    /**
     * @brief Callback for handling incoming media streams
     */
    static void on_incoming_stream(GstElement* webrtcbin, GstPad* pad, WebRTCSend* self) {
        LOG_INFO("Received incoming stream");

        // Create decoder for incoming stream
        GstElement* decodebin = gst_element_factory_make("decodebin", nullptr);
        if (!decodebin) {
            LOG_ERROR("Could not create decodebin element");
            return;
        }
        
        gst_bin_add(GST_BIN(self->pipeline), decodebin);
        gst_element_sync_state_with_parent(decodebin);

        // Connect decoder pad-added signal
        g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), self);

        // Link incoming pad to decoder
        GstPad* sinkpad = gst_element_get_static_pad(decodebin, "sink");
        gst_pad_link(pad, sinkpad);
        gst_object_unref(sinkpad);
    }

    /**
     * @brief Callback when decodebin creates a new pad
     * 
     * Sets up appropriate elements for handling decoded audio/video streams
     */
    static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, WebRTCSend* self) {
        // Get pad capabilities
        GstCaps* caps = gst_pad_get_current_caps(pad);
        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* media_type = gst_structure_get_name(str);

        // Create appropriate elements based on media type
        GstElement* conv = nullptr;
        GstElement* sink = nullptr;

        if (g_str_has_prefix(media_type, "video")) {
            LOG_INFO("Setting up video display pipeline");
            conv = gst_element_factory_make("videoconvert", nullptr);
            sink = gst_element_factory_make("autovideosink", nullptr);
        } else if (g_str_has_prefix(media_type, "audio")) {
            LOG_INFO("Setting up audio playback pipeline");
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        } else {
            LOG_INFO("Unknown media type: " + std::string(media_type));
            gst_caps_unref(caps);
            return;
        }

        // Add and link elements if created successfully
        if (conv && sink) {
            gst_bin_add_many(GST_BIN(self->pipeline), conv, sink, nullptr);
            gst_element_sync_state_with_parent(conv);
            gst_element_sync_state_with_parent(sink);

            // Link decoded pad to converter
            GstPad* sinkpad = gst_element_get_static_pad(conv, "sink");
            gst_pad_link(pad, sinkpad);
            gst_object_unref(sinkpad);

            // Link converter to sink
            GstPad* srcpad = gst_element_get_static_pad(conv, "src");
            GstPad* sinkpad2 = gst_element_get_static_pad(sink, "sink");
            gst_pad_link(srcpad, sinkpad2);
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad2);
        } else {
            LOG_ERROR("Failed to create media processing elements");
        }

        gst_caps_unref(caps);
    }

private:
    GstElement* pipeline;                // Main GStreamer pipeline
    GstElement* webrtcbin;               // WebRTC element
    ConnectionHandle global_hdl;         // WebSocket connection handle
    WebSocketClient client;              // WebSocket client
    
    std::queue<std::string> msg_queue;   // Queue for outgoing WebSocket messages
    std::mutex ws_mutex;                 // Mutex for thread safety
    std::condition_variable ws_cv;       // Condition variable for message queue
    std::thread ws_thread;               // Thread for processing WebSocket messages
    std::atomic<bool> ws_running;        // Flag to control WebSocket thread
};

// Global pointer for signal handling
static WebRTCSend* g_sender = nullptr;

/**
 * @brief Signal handler for graceful shutdown
 */
void signal_handler(int signum) {
    std::cout << "Caught signal " << signum << ", exiting..." << std::endl;
    if (g_sender) {
        g_sender->cleanup();
    }
    exit(signum);
}

/**
 * @brief Main entry point
 * 
 * Creates WebRTCSend instance and runs the main loop
 */
int main(int argc, char* argv[]) {
    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Default connection parameters
    std::string server_address = "0.0.0.0";  // Default to localhost
    int server_port = 8765;                  // Default port
    
    // Parse command line arguments for server address and port
    if (argc > 1) {
        server_address = argv[1];
    }
    if (argc > 2) {
        try {
            server_port = std::stoi(argv[2]);
            if (server_port <= 0 || server_port > 65535) {
                LOG_ERROR("Invalid port number. Using default port 8765");
                server_port = 8765;
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to parse port number: " + std::string(e.what()));
            LOG_ERROR("Using default port 8765");
        }
    }
    
    LOG_INFO("WebRTC Video Sender for Raspberry Pi");
    LOG_INFO("====================================");
    LOG_INFO("Connecting to signaling server at: ws://" + server_address + ":" + std::to_string(server_port));

    // Create sender instance and start connection
    WebRTCSend sender;
    g_sender = &sender;  // Store for signal handler
    
    // Run connection in a separate thread
    std::thread t([&sender, server_address, server_port]() { 
        sender.connect(server_address, server_port); 
    });

    // Run GLib main loop
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    // Wait for connection thread to finish
    t.join();
    
    g_sender = nullptr;
    return 0;
} 