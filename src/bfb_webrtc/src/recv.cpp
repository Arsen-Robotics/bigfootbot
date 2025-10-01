// Required headers
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/gstsdp.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <X11/Xlib.h>
#include <glib.h>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief Main class handling WebRTC video streaming
 * 
 * This class manages the WebRTC connection, GStreamer pipeline,
 * and WebSocket signaling to stream video from multiple cameras
 */
class WebRTCRecvNode : public rclcpp::Node {
public:
    // Flag to control threads
    std::atomic<bool> running{true};

    /**
     * @brief Constructor - initializes GStreamer and member variables
     */
    WebRTCRecvNode() : Node("webrtc_recv_node") {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        pipeline = nullptr;
        webrtcbin = nullptr;
        running = true;
    }

    /**
     * @brief Destructor - cleans up resources
     */
    ~WebRTCRecvNode() {
        running = false;
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

            client.set_open_handler(std::bind(&WebRTCRecvNode::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCRecvNode::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection("ws://87.119.173.184:8765", ec);

            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Connection error: %s", ec.message().c_str());
            }

            client.connect(con);
            
            while (rclcpp::ok() && running) {
                client.poll();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", e.what());
        }
    }

    /**
     * @brief Handler called when WebSocket connection is established
     * 
     * @param hdl Connection handle
     * @param c Pointer to WebSocket client
     */
    void on_open(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>* c) {
        RCLCPP_INFO(this->get_logger(), "Connected! Sending HELLO message...");

        global_hdl = hdl;
        global_client = c;

        ws_thread = std::thread(&WebRTCRecvNode::process_ws_queue, this);
        ws_thread.detach();

        // Create a JSON object to send "HELLO" message
        Json::Value msg;
        msg["status"] = "HELLO";

        // Convert to string
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);

        // Send the HELLO message
        queue_ws(message);
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
                RCLCPP_ERROR(this->get_logger(), "Error: JSON message is not an object");
                return;
            }

            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "OK") {
                RCLCPP_INFO(this->get_logger(), "Received OK.");

                // Post pipeline setup to main thread
                auto setup_func = [this]() { this->setup_pipeline(); };
                auto* func_ptr = new std::function<void()>(setup_func);
                g_main_context_invoke(nullptr, [](gpointer data) -> gboolean {
                    auto* f = static_cast<std::function<void()>*>(data);
                    (*f)();
                    delete f;
                    return G_SOURCE_REMOVE;
                }, func_ptr);

            } else if (jsonMsg.isMember("ice")) {
                RCLCPP_INFO(this->get_logger(), "Received ICE candidate.");
                // Copy payload for lambda capture
                std::string payload_copy = payload;
                auto ice_func = [this, payload_copy]() { this->handle_ice(payload_copy); };
                auto* func_ptr = new std::function<void()>(ice_func);
                g_main_context_invoke(nullptr, [](gpointer data) -> gboolean {
                    auto* f = static_cast<std::function<void()>*>(data);
                    (*f)();
                    delete f;
                    return G_SOURCE_REMOVE;
                }, func_ptr);

            } else if (jsonMsg.isMember("sdp")) {
                RCLCPP_INFO(this->get_logger(), "Received SDP offer.");
                std::string payload_copy = payload;
                auto sdp_func = [this, payload_copy]() { this->handle_sdp(payload_copy); };
                auto* func_ptr = new std::function<void()>(sdp_func);
                g_main_context_invoke(nullptr, [](gpointer data) -> gboolean {
                    auto* f = static_cast<std::function<void()>*>(data);
                    (*f)();
                    delete f;
                    return G_SOURCE_REMOVE;
                }, func_ptr);

            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown JSON message type");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", errs.c_str());
        }
    }

    /**
     * @brief Processes queued WebSocket messages
     * 
     * Runs in a separate thread to handle outgoing WebSocket messages
     */
    void process_ws_queue() {
        while (rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(ws_mutex);
            ws_cv.wait(lock, [this] { return !msg_queue.empty() || !running; });

            std::string msg = msg_queue.front();
            msg_queue.pop();
            lock.unlock();

            websocketpp::lib::error_code ec;
            global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Error sending WebSocket message: %s", ec.message().c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent message over WebSocket: %s", msg.c_str());
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
     * - Multiple v4l2src sources for different cameras
     * - NVIDIA video conversion and H264 encoding
     * - WebRTC transmission
     */
    void setup_pipeline() {
        // Create GStreamer pipeline
        GError* error = nullptr;
        pipeline = gst_parse_launch("webrtcbin name=recv stun-server=stun://stun.l.google.com:19302 \
                videotestsrc is-live=true pattern=black ! video/x-raw,width=16,height=16,framerate=1/1 ! videoconvert ! fakesink",
            &error);

        if (error) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create GStreamer pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        if (!pipeline) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create GStreamer pipeline.");
            return;
        }

        // Get webrtcbin element
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "recv");
        if (!webrtcbin) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not get WebRTC element.");
            return;
        }

        gst_pipeline_use_clock(GST_PIPELINE(pipeline), gst_system_clock_obtain());

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCRecvNode::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCRecvNode::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    /**
     * @brief Callback for sending ICE candidates
     * 
     * Called when a new ICE candidate is discovered
     */
    static void send_ice_candidate(GstElement* webrtcbin, guint mlineindex, gchar* candidate, gpointer user_data) {
        auto * self = static_cast<WebRTCRecvNode*>(user_data);
        RCLCPP_INFO(self->get_logger(), "Sending ICE candidate: %s", candidate);

        // Create a JSON message to send over WebSocket
        Json::Value icemsg;
        icemsg["ice"]["candidate"] = candidate;
        icemsg["ice"]["sdpMLineIndex"] = static_cast<int>(mlineindex);  // Casting mlineindex to int for JSON

        // Convert JSON message to string
        Json::StreamWriterBuilder writer;
        std::string icemsg_str = Json::writeString(writer, icemsg);

        // Send the ICE candidate over WebSocket
        static_cast<WebRTCRecvNode*>(user_data)->queue_ws(icemsg_str);
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
                RCLCPP_ERROR(this->get_logger(), "ERROR: Could not get WebRTC element.");
                return;
            }

            g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse ICE candidate: %s", errs.c_str());
        }
    }

    /**
     * @brief Handles incoming SDP messages
     * 
     * @param sdp JSON string containing SDP information
     */
    void handle_sdp_offer(const std::string& sdp) {
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(sdp);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            std::string sdpType = jsonMsg["sdp"]["type"].asString();
            std::string sdp_str = jsonMsg["sdp"]["sdp"].asString();

            if (sdpType == "offer") {
                GstSDPMessage* sdp_msg = nullptr;
                gst_sdp_message_new(&sdp_msg);
                GstSDPResult result = gst_sdp_message_parse_buffer((guint8*)sdp_str.c_str(), sdp_str.size(), sdp_msg);

                if (result != GST_SDP_OK) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse SDP message: %d", result);
                    return;
                }

                GstWebRTCSessionDescription* offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_msg);
                if (!offer) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create WebRTC session description");
                    gst_sdp_message_free(sdp_msg);
                    return;
                }

                g_signal_emit_by_name(webrtcbin, "set-remote-description", offer, nullptr);
                gst_webrtc_session_description_free(offer);

                // Create answer
                GstPromise *promise = gst_promise_new_with_change_func([](GstPromise *promise, gpointer user_data) {
                    WebRTCRecv* self = static_cast<WebRTCRecv*>(user_data);
                    GstStructure const *reply = gst_promise_get_reply(promise);
                    
                    GstWebRTCSessionDescription *answer = nullptr;
                    gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
                    gst_promise_unref(promise);

                    if (!answer) {
                        RCLCPP_ERROR(self->get_logger(), "Failed to create SDP answer.");
                        return;
                    }

                    self->on_answer_created(answer);
                }, this, nullptr);

                g_signal_emit_by_name(webrtcbin, "create-answer", nullptr, promise);

            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported SDP type: %s", sdpType.c_str());
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse SDP: %s", errs.c_str());
        }
    }

    /**
     * @brief Callback when SDP answer is created
     * 
     * @param answer Created SDP answer
     */
    void on_answer_created(GstWebRTCSessionDescription* answer) {
        // Set local description
        g_signal_emit_by_name(webrtcbin, "set-local-description", answer, nullptr);

        // Convert SDP answer to string
        gchar* sdpStr = gst_sdp_message_as_text(answer->sdp);
        Json::Value answerMsg;
        answerMsg["sdp"]["type"] = "answer";
        answerMsg["sdp"]["sdp"] = sdpStr;
        Json::StreamWriterBuilder writer;

        // Send the SDP answer over WebSocket
        std::string answerStr = Json::writeString(writer, answerMsg);
        queue_ws(answerStr);
        g_free(sdpStr);
        gst_webrtc_session_description_free(answer);
    }

    /**
     * @brief Callback for handling incoming media streams
     */
    static void on_incoming_stream(GstElement* webrtcbin, GstPad* pad, WebRTCRecvNode* self) {
        RCLCPP_INFO(self->get_logger(), "Received incoming stream.");

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
     * @param decodebin The decodebin element
     * @param pad The newly created pad
     * @param self Pointer to the WebRTCRecvNode instance
     * 
     * Sets up appropriate elements for handling decoded audio/video streams
     */
    static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, WebRTCRecvNode* self) {
        GstClock* clock = gst_system_clock_obtain();
        gst_pipeline_use_clock(GST_PIPELINE(self->pipeline), clock);
        g_object_set(self->pipeline, "latency", 0, NULL); // 1ms target latency

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
                "max-size-buffers", 20, // yesterday was 20 when testing outside
                "max-size-time", 0,
                "max-size-bytes", 0,
                "leaky", 2, // downstream
                NULL);
            
            conv = gst_element_factory_make("videoconvert", nullptr);

            sink = gst_element_factory_make("xvimagesink", nullptr);
            g_object_set(sink,
                "sync", FALSE,
                "max-lateness", 100000000,  // Drop immediately if late
                NULL);

        } else if (g_str_has_prefix(name, "audio")) {
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        }

        // Add elements to the pipeline
        gst_bin_add_many(GST_BIN(self->pipeline), queue, conv, sink, nullptr);
        gst_element_sync_state_with_parent(queue);
        gst_element_sync_state_with_parent(conv);
        gst_element_sync_state_with_parent(sink);

        // Link: decodebin pad → queue → conv → sink
        GstPad* sinkpad = gst_element_get_static_pad(queue, "sink");
        gst_pad_link(pad, sinkpad);
        gst_object_unref(sinkpad);

        gst_element_link_many(queue, conv, sink, nullptr);
        gst_caps_unref(caps);
    }

private:
    std::thread ws_thread;
    GstElement* pipeline;
    GstElement* webrtcbin;
    websocketpp::connection_hdl global_hdl;
    websocketpp::client<websocketpp::config::asio_client>* global_client;
    
    std::queue<std::string> msg_queue;
    std::mutex ws_mutex;
    std::condition_variable ws_cv;
};

/**
 * @brief Main entry point
 * 
 * Initializes X11 threading, creates WebRTCRecvNode instance,
 * and runs the main loop
 */
int main(int argc, char* argv[]) {
    if (!XInitThreads()) {
        RCLCPP_FATAL(rclcpp::get_logger("WebRTCRecvNode"), "Failed to initialize X11 threading!");
        return 1;
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebRTCSRecvNode>();

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);

    // Start GStreamer loop first, then WebSocket connection
    std::thread gloop_thread([&]() { 
        g_main_loop_run(loop); 
    });

    // Start WebSocket connect in separate thread
    std::thread ws_connect_thread([&]() { 
        node->connect(); 
    });

    rclcpp::spin(node);

    // Cleanup
    node->running = false;
    ws_connect_thread.join();
    rclcpp::shutdown();
    return 0;
}