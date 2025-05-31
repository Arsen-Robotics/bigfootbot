/**
 * @file recv.cpp
 * @brief WebRTC video receiver implementation
 *
 * This file implements a WebRTC video receiver that connects to a signaling server,
 * establishes WebRTC connections, and displays received video streams.
 */

// Required headers
#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <X11/Xlib.h>

/**
 * @brief Main class handling WebRTC video receiving
 * 
 * This class manages the WebRTC connection, GStreamer pipeline,
 * and WebSocket signaling to receive and display video streams
 */
class WebRTCRecv {
public:
    /**
     * @brief Constructor - initializes GStreamer and member variables
     */
    WebRTCRecv() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        this->pipeline = nullptr;
        this->webrtcbin = nullptr;
    }

    /**
     * @brief Destructor - cleans up resources
     */
    ~WebRTCRecv() {
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
     */
    void connect() {
        try {
            // Set up WebSocket connection using websocketpp
            websocketpp::client<websocketpp::config::asio_client> client;
            client.init_asio();

            client.set_open_handler(std::bind(&WebRTCRecv::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCRecv::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection("ws://87.119.173.184:8765", ec);

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
     * @brief Handler for WebSocket connection open event
     * 
     * @param hdl WebSocket connection handle
     * @param c WebSocket client pointer
     */
    void on_open(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>* c) {
        std::cout << "Connected! Sending HELLO message..." << std::endl;

        global_hdl = hdl;
        global_client = c;

        //ws_thread = std::thread(&WebRTCRecv::process_ws_queue, this);

        // Create a JSON object to send "HELLO" message
        Json::Value msg;
        msg["status"] = "HELLO";

        // Convert to string
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);

        // Send the initial message
        send_ws(message);
    }

    /**
     * @brief Handler for incoming WebSocket messages
     * 
     * Processes different types of messages including ICE candidates and SDP offers
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

            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "OK") {
                std::cout << "Received OK status." << std::endl;
                setup_pipeline();
            } else if (jsonMsg.isMember("ice")) {
                std::cout << "Received ICE candidate." << std::endl;
                handle_ice(payload);
            } else if (jsonMsg.isMember("sdp")) {
                std::cout << "Received SDP offer." << std::endl;
                handle_sdp(payload);
            } else {
                std::cout << "Unknown JSON message type" << std::endl;
            }
        } else {
            std::cout << "Failed to parse JSON: " << errs << std::endl;
        }
    }
    
    // Commented out WebSocket queue processing code
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

    /**
     * @brief Sends a WebSocket message
     * 
     * @param msg Message to send
     */
    void send_ws(const std::string& msg) {
        websocketpp::lib::error_code ec;
        global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
        if (ec) {
            std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
        } else {
            std::cout << "Sent message over WebSocket: " << msg << std::endl;
        }
    }

    // Commented out WebSocket queue code
    // void queue_ws(const std::string& msg) {
    //     std::lock_guard<std::mutex> lock(ws_mutex);
    //     msg_queue.push(msg);
    //     ws_cv.notify_one();
    // }

    /**
     * @brief Sets up the GStreamer pipeline for WebRTC
     */
    void setup_pipeline() {
        // Create GStreamer pipeline
        GError* error = nullptr;
        pipeline = gst_parse_launch("webrtcbin name=recvonly stun-server=stun://stun.l.google.com:19302 audiotestsrc ! audioconvert ! fakesink", &error);
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
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "recvonly");

        if (!webrtcbin) {
            std::cerr << "ERROR: Could not get WebRTC element." << std::endl;
            return;
        }

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "latency", 0, "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCRecv::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCRecv::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }
    
    /**
     * @brief Callback for sending ICE candidates
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
        static_cast<WebRTCRecv*>(user_data)->send_ws(icemsg_str);
    }

    /**
     * @brief Handles incoming ICE candidates
     * 
     * @param ice ICE candidate message
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
     * @brief Handles incoming SDP messages
     * 
     * @param sdp SDP message
     */
    void handle_sdp(const std::string& sdp) {
        std::cout << "Handling SDP offer." << std::endl;
        
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(sdp);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            std::string sdpType = jsonMsg["sdp"]["type"].asString();
            std::string sdpDescription = jsonMsg["sdp"]["sdp"].asString();

            if (sdpType == "offer") {
                GstSDPMessage* sdpMsg = nullptr;
                gst_sdp_message_new(&sdpMsg);
                gst_sdp_message_parse_buffer((guint8*)sdpDescription.c_str(), sdpDescription.size(), sdpMsg);

                GstWebRTCSessionDescription* offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdpMsg);
                g_signal_emit_by_name(webrtcbin, "set-remote-description", offer, nullptr);

                // Create answer immediately
                GstPromise *promise = gst_promise_new_with_change_func([](GstPromise *promise, gpointer user_data) {
                    WebRTCRecv* self = static_cast<WebRTCRecv*>(user_data);
                    GstStructure const *reply = gst_promise_get_reply(promise);
                    
                    GstWebRTCSessionDescription *answer = nullptr;
                    gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
                    gst_promise_unref(promise);

                    if (!answer) {
                        std::cerr << "Failed to create SDP answer." << std::endl;
                        return;
                    }

                    self->on_answer_created(answer);
                }, this, nullptr);

                g_signal_emit_by_name(webrtcbin, "create-answer", nullptr, promise);

            } else {
                std::cerr << "Unsupported SDP type: " << sdpType << std::endl;
            }
        } else {
            std::cerr << "Failed to parse SDP: " << errs << std::endl;
        }
    }

    /**
     * @brief Handles created SDP answer
     * 
     * @param answer Created SDP answer
     */
    void on_answer_created(GstWebRTCSessionDescription* answer) {
        std::cout << "Answer created, setting local description and sending..." << std::endl;

        g_signal_emit_by_name(webrtcbin, "set-local-description", answer, nullptr);

        gchar* sdpStr = gst_sdp_message_as_text(answer->sdp);

        Json::Value answerMsg;
        answerMsg["sdp"]["type"] = "answer";
        answerMsg["sdp"]["sdp"] = sdpStr;

        Json::StreamWriterBuilder writer;
        std::string answerStr = Json::writeString(writer, answerMsg);

        send_ws(answerStr);
        g_free(sdpStr);
        gst_webrtc_session_description_free(answer);
    }

    /**
     * @brief Callback for handling incoming media streams
     */
    static void on_incoming_stream(GstElement* webrtcbin, GstPad* pad, WebRTCRecv* self) {
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
     * @brief Callback for handling decoded media pads
     * 
     * Sets up appropriate elements for handling decoded audio/video streams
     */
    static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, WebRTCRecv* self) {
        GstCaps* caps = gst_pad_get_current_caps(pad);
        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);

        GstElement* conv = nullptr;
        GstElement* sink = nullptr;
        GstElement* queue = nullptr;

        if (g_str_has_prefix(name, "video")) {
            // Create queue to help absorb jitter
            queue = gst_element_factory_make("queue", nullptr);
            g_object_set(queue,
                "max-size-buffers", 15,
                "max-size-time", 0,
                "max-size-bytes", 0,
                "leaky", 2, // downstream
                NULL);

            conv = gst_element_factory_make("videoconvert", nullptr);
            sink = gst_element_factory_make("xvimagesink", nullptr);
            g_object_set(sink, "sync", FALSE, NULL);
        } else if (g_str_has_prefix(name, "audio")) {
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        }

        if (conv && sink) {
            if (queue) {
                gst_bin_add_many(GST_BIN(self->pipeline), queue, conv, sink, nullptr);
                gst_element_sync_state_with_parent(queue);
            } else {
                gst_bin_add_many(GST_BIN(self->pipeline), conv, sink, nullptr);
            }

            gst_element_sync_state_with_parent(conv);
            gst_element_sync_state_with_parent(sink);

            if (queue) {
                // Link: decodebin pad → queue → conv → sink
                GstPad* sinkpad = gst_element_get_static_pad(queue, "sink");
                gst_pad_link(pad, sinkpad);
                gst_object_unref(sinkpad);

                gst_element_link_many(queue, conv, sink, nullptr);
            } else {
                // Link: decodebin pad → conv → sink (no queue)
                GstPad* sinkpad = gst_element_get_static_pad(conv, "sink");
                gst_pad_link(pad, sinkpad);
                gst_object_unref(sinkpad);

                gst_element_link(conv, sink);
            }
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
 * Initializes X11 threading, creates WebRTCRecv instance,
 * and starts the connection
 */
int main() {
    // Call XInitThreads() to enable thread safety in X11
    if (!XInitThreads()) {
        std::cerr << "Failed to initialize X11 threading!" << std::endl;
        return 1;
    }

    WebRTCRecv recv;
    recv.connect();

    return 0;
}