// Required headers
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
        ws_running = true;
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
     */
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
                Json::Value reply;
                reply["status"] = "OK";
                Json::StreamWriterBuilder writer;
                std::string message = Json::writeString(writer, reply);
                queue_ws(message);

                // Post pipeline setup to main thread
                g_idle_add([](gpointer data) -> gboolean {
                    static_cast<WebRTCSend*>(data)->setup_pipeline();
                    return G_SOURCE_REMOVE;
                }, this);

            } else if (jsonMsg.isMember("ice")) {
                std::cout << "Received ICE candidate." << std::endl;
                // Copy payload for lambda capture
                std::string payload_copy = payload;
                g_idle_add([](gpointer data) -> gboolean {
                    auto* pair = static_cast<std::pair<WebRTCSend*, std::string>*>(data);
                    pair->first->handle_ice(pair->second);
                    delete pair;
                    return G_SOURCE_REMOVE;
                }, new std::pair<WebRTCSend*, std::string>(this, payload_copy));

            } else if (jsonMsg.isMember("sdp")) {
                std::cout << "Received SDP answer." << std::endl;
                std::string payload_copy = payload;
                g_idle_add([](gpointer data) -> gboolean {
                    auto* pair = static_cast<std::pair<WebRTCSend*, std::string>*>(data);
                    pair->first->handle_sdp(pair->second);
                    delete pair;
                    return G_SOURCE_REMOVE;
                }, new std::pair<WebRTCSend*, std::string>(this, payload_copy));

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

    // Commented out direct send method in favor of queued approach
    // void send_ws(const std::string& msg) {
    //     websocketpp::lib::error_code ec;
    //     global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
    //     if (ec) {
    //         std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
    //     } else {
    //         std::cout << "Sent message over WebSocket: " << msg << std::endl;
    //     }
    // }

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
        pipeline = gst_parse_launch("webrtcbin name=sendrecv bundle-policy=max-bundle latency=0 \
            stun-server=stun://stun.l.google.com:19302 \
            v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! queue max-size-buffers=2 leaky=downstream \
            ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! queue max-size-buffers=2 leaky=downstream \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            \
            nvarguscamerasrc sensor-mode=4 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! queue max-size-buffers=2 leaky=downstream \
            ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! queue max-size-buffers=2 leaky=downstream \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            \
            v4l2src device=/dev/cam-aveo ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            ! queue max-size-buffers=2 leaky=downstream \
            ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            ! queue max-size-buffers=2 leaky=downstream \
            ! h264parse ! rtph264pay config-interval=1 pt=96 \
            ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.",
            &error);

            // v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 \
            // ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            // ! nvv4l2h264enc bitrate=3000000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            // ! h264parse ! rtph264pay config-interval=1 pt=96 \
            // ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            // \
            // v4l2src device=/dev/cam-microdia ! video/x-raw,width=640,height=480,framerate=30/1 \
            // ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            // ! nvv4l2h264enc bitrate=3000000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            // ! h264parse ! rtph264pay config-interval=1 pt=96 \
            // ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv. \
            // \
            // nvarguscamerasrc sensor-mode=4 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 \
            // ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
            // ! nvv4l2h264enc bitrate=3000000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
            // ! h264parse ! rtph264pay config-interval=1 pt=96 \
            // ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! sendrecv.

            // nvcompositor name=mix sync-import-streams=false \
            // sink_0::xpos=0   sink_0::ypos=0   sink_0::width=640  sink_0::height=480 \
            // sink_1::xpos=640 sink_1::ypos=0   sink_1::width=640  sink_1::height=480 \
            // sink_2::xpos=1280 sink_2::ypos=0  sink_2::width=640  sink_2::height=480 \
            // ! queue max-size-buffers=7 leaky=upstream \
            // ! video/x-raw(memory:NVMM), width=1920, height=480, framerate=30/1 \
            // ! nvvidconv \
            // ! video/x-raw(memory:NVMM), format=NV12 \
            // ! queue max-size-buffers=7 leaky=upstream \
            // ! nvv4l2h264enc maxperf-enable=true bitrate=4000000 idrinterval=5 iframeinterval=5 insert-sps-pps=true insert-aud=true insert-vui=true \
            // ! queue max-size-buffers=7 leaky=upstream \
            // ! h264parse config-interval=1 \
            // ! rtph264pay pt=96 mtu=1200 config-interval=1 \
            // ! sendrecv. \
            // \
            // nvarguscamerasrc sensor-mode=4 \
            // ! queue max-size-buffers=7 leaky=upstream \
            // ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 \
            // ! nvvidconv \
            // ! video/x-raw(memory:NVMM),format=NV12 \
            // ! mix.sink_0 \
            // \
            // v4l2src device=/dev/video9 io-mode=4 \
            // ! video/x-raw,width=640,height=480,framerate=30/1 \
            // ! nvvidconv \
            // ! video/x-raw(memory:NVMM),format=NV12 \
            // ! mix.sink_1 \
            // \
            // v4l2src device=/dev/video7 io-mode=4 \
            // ! video/x-raw,width=640,height=480,framerate=30/1 \
            // ! nvvidconv \
            // ! video/x-raw(memory:NVMM),format=NV12 \
            // ! mix.sink_2", &error);

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

        gst_pipeline_use_clock(GST_PIPELINE(pipeline), gst_system_clock_obtain());

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(&WebRTCSend::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCSend::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCSend::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
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
 * Initializes X11 threading, creates WebRTCSend instance,
 * and runs the main loop
 */
int main() {
    if (!XInitThreads()) {
        std::cerr << "Failed to initialize X11 threading!" << std::endl;
        return 1;
    }

    WebRTCSend recv;
    std::thread ws_thread([&]() { recv.connect(); });

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    ws_thread.join();
    return 0;
}