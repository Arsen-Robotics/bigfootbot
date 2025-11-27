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
#include <sensor_msgs/msg/image.hpp>
#include <sys/resource.h>

static GQuark STREAM_INFO_QUARK = 0;

/**
 * @brief Main class handling WebRTC video streaming
 * 
 * This class manages the WebRTC connection, GStreamer pipeline,
 * and WebSocket signaling to stream video from multiple cameras
 */
class WebRTCSendNode : public rclcpp::Node {
public:
    // Flag to control ws_thread
    std::atomic<bool> ws_running{true};

    // Flag to control bitrate_thread
    // std::atomic<bool> bitrate_running{false};

    /**
     * @brief Constructor - initializes GStreamer and member variables
     */
    WebRTCSendNode() : Node("webrtc_send_node") {
        // Set this process to higher priority
        if (setpriority(PRIO_PROCESS, 0, -10) == 0) {
            RCLCPP_INFO(this->get_logger(), "Running at high priority.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not set priority.");
        }

        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        pipeline = nullptr;
        webrtcbin = nullptr;
        ws_running = true;
        STREAM_INFO_QUARK = g_quark_from_static_string("stream-info");

        // Frame interval monitor callback
        frame_interval_monitor_timer = this->create_wall_timer(
            std::chrono::milliseconds(frame_interval_monitor_period_ms),
            [this]() { this->frame_interval_monitor_callback(); }
        );
    }

    /**
     * @brief Destructor - cleans up resources
     */
    ~WebRTCSendNode() {
        // Stop WebSocket thread
        ws_running = false;
        ws_cv.notify_all();

        if (ws_thread.joinable()) {
            ws_thread.join();
        }

        // Stop bitrate controller
        // bitrate_running = false;
        // if (bitrate_thread.joinable()) {
        //     bitrate_thread.join();
        // }

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

            // Disable WebSocket++ logs
            client.clear_access_channels(websocketpp::log::alevel::all);

            // Optionally, keep only connection logs (if you still want to know when connected/disconnected):
            client.set_access_channels(websocketpp::log::alevel::connect | websocketpp::log::alevel::disconnect);

            client.set_open_handler(std::bind(&WebRTCSendNode::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCSendNode::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection("ws://0.0.0.0:8765", ec);

            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Connection error: %s", ec.message().c_str());
            }

            client.connect(con);
            
            while (rclcpp::ok() && ws_running) {
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
        RCLCPP_INFO(this->get_logger(), "Connected! Waiting for HELLO message...");

        global_hdl = hdl;
        global_client = c;

        ws_thread = std::thread(&WebRTCSendNode::process_ws_queue, this);
        ws_thread.detach();
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

            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "HELLO") {
                RCLCPP_INFO(this->get_logger(), "Received HELLO.");
                Json::Value reply;
                reply["status"] = "OK";
                Json::StreamWriterBuilder writer;
                std::string message = Json::writeString(writer, reply);
                queue_ws(message);

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
                RCLCPP_INFO(this->get_logger(), "Received SDP answer.");
                std::string payload_copy = payload;
                auto sdp_func = [this, payload_copy]() { this->handle_sdp_answer(payload_copy); };
                auto* func_ptr = new std::function<void()>(sdp_func);
                g_main_context_invoke(nullptr, [](gpointer data) -> gboolean {
                    auto* f = static_cast<std::function<void()>*>(data);
                    (*f)();
                    delete f;
                    return G_SOURCE_REMOVE;
                }, func_ptr);

            } else if (jsonMsg.isMember("control")) {
                const auto& ctrl = jsonMsg["control"];
                std::string action = ctrl["action"].asString();

                if (action == "change_bitrate") {
                    int stream_id = ctrl["stream_id"].asInt();
                    int delta_bitrate = ctrl["delta"].asInt();
                    change_bitrate(stream_id, delta_bitrate);
                }

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
            ws_cv.wait(lock, [this] { return !msg_queue.empty() || !ws_running; });

            std::string msg = msg_queue.front();
            msg_queue.pop();
            lock.unlock();

            websocketpp::lib::error_code ec;
            global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Error sending WebSocket message: %s", ec.message().c_str());
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

    void add_stream(const std::string& src_type,
                    const std::string& device,
                    const std::string& format,
                    int width,
                    int height,
                    int framerate,
                    int bitrate) {
        // Create GStreamer elements for the stream
        GstElement *src, *src_queue, *src_capsfilter, *jpegdec_stage, *conv, *conv_capsfilter,
        *videorate, *videorate_capsfilter, *enc_queue0, *enc, *enc_queue1, *parse, *pay, *pay_capsfilter *rtpulpfecenc;

        // Source
        if (src_type == "v4l2src") {
            src = gst_element_factory_make("v4l2src", NULL);
            g_object_set(G_OBJECT(src), "device", device.c_str(), "io-mode", 4, NULL);
        } else if (src_type == "argus") {
            src = gst_element_factory_make("nvarguscamerasrc", NULL);
            g_object_set(G_OBJECT(src), "sensor-mode", 3, NULL);
        }

        // Get the src pad of source element
        GstPad* src_srcpad = gst_element_get_static_pad(src, "src");

        // Caps filter - after camera source
        src_capsfilter = gst_element_factory_make("capsfilter", NULL);
        if (src_type == "v4l2src") {
            GstCaps* src_caps = gst_caps_new_simple(format.c_str(),
                                                "width", G_TYPE_INT, width,
                                                "height", G_TYPE_INT, height,
                                                "framerate", GST_TYPE_FRACTION, framerate, 1,
                                                NULL);
            g_object_set(G_OBJECT(src_capsfilter), "caps", src_caps, NULL);
            gst_caps_unref(src_caps);
        } else if (src_type == "argus") {
            GstCaps* src_caps = gst_caps_new_simple(format.c_str(),
                                                "width", G_TYPE_INT, width,
                                                "height", G_TYPE_INT, height,
                                                "framerate", GST_TYPE_FRACTION, framerate, 1,
                                                NULL);

            // Set memory feature to NVMM
            GstCapsFeatures* argus_src_features = gst_caps_features_new("memory:NVMM", NULL);
            gst_caps_set_features(src_caps, 0, argus_src_features);

            g_object_set(G_OBJECT(src_capsfilter), "caps", src_caps, NULL);
            gst_caps_unref(src_caps);
        }

        // Queue - after camera source
        src_queue = gst_element_factory_make("queue", NULL);
        g_object_set(G_OBJECT(src_queue), "max-size-buffers", 15, "leaky", 2, NULL); // downstream leaky
    
        // JPEG decoder (only create if stream is JPEG)
        if (format == "image/jpeg") {
            jpegdec_stage = gst_element_factory_make("nvv4l2decoder", NULL);
            g_object_set(G_OBJECT(jpegdec_stage), "mjpeg", 1, NULL);
        } else {
            jpegdec_stage = gst_element_factory_make("identity", NULL); // do nothing
        }

        // Converter
        conv = gst_element_factory_make("nvvidconv", NULL);

        // Caps filter - after converter
        conv_capsfilter = gst_element_factory_make("capsfilter", NULL);
        GstCaps* conv_caps = gst_caps_new_simple("video/x-raw",
                                             "format", G_TYPE_STRING, "NV12",
                                             NULL);

        // Set memory feature to NVMM
        GstCapsFeatures* conv_features = gst_caps_features_new("memory:NVMM", NULL);
        gst_caps_set_features(conv_caps, 0, conv_features);

        g_object_set(G_OBJECT(conv_capsfilter), "caps", conv_caps, NULL);
        gst_caps_unref(conv_caps);

        // Videorate
        videorate = gst_element_factory_make("videorate", NULL);

        // Capsfilter - after videorate
        videorate_capsfilter = gst_element_factory_make("capsfilter", NULL);
        GstCaps* videorate_caps = gst_caps_new_simple("video/x-raw",
                                             "format", G_TYPE_STRING, "NV12",
                                             "drop-only", G_TYPE_BOOLEAN, TRUE,
                                             NULL);
        
        // Set memory feature to NVMM
        GstCapsFeatures* videorate_features = gst_caps_features_new("memory:NVMM", NULL);
        gst_caps_set_features(videorate_caps, 0, videorate_features);

        g_object_set(G_OBJECT(videorate_capsfilter), "caps", videorate_caps, NULL);
        gst_caps_unref(videorate_caps);

        // Queue - before encoder
        enc_queue0 = gst_element_factory_make("queue", NULL);
        g_object_set(G_OBJECT(enc_queue0), "max-size-buffers", 10, "leaky", 2, NULL); // downstream leaky

        // Encoder
        std::string enc_name = "enc" + std::to_string(pipeline_stream_index);
        enc = gst_element_factory_make("nvv4l2h264enc", enc_name.c_str());
        g_object_set(enc,
                 "control-rate", 1,
                 "bitrate", bitrate,
                 "iframeinterval", 15,
                 "num-B-Frames", 0,
                 "preset-level", 1,
                 "profile", 0,
                 "maxperf-enable", 1,
                 "insert-sps-pps", 1,
                 "insert-vui", 1,
                 "EnableTwopassCBR", 0,
                 NULL);

        // Queue - after encoder
        enc_queue1 = gst_element_factory_make("queue", NULL);
        g_object_set(G_OBJECT(enc_queue1), "max-size-buffers", 5, "leaky", 2, NULL); // downstream leaky

        // Parser
        parse = gst_element_factory_make("h264parse", NULL);
        g_object_set(parse, "config-interval", 0, NULL);

        // Payloader
        pay = gst_element_factory_make("rtph264pay", NULL);
        g_object_set(pay,
                    "pt", 96,
                    "mtu", 1200,
                    "config-interval", -1,
                    NULL);                 

        // Caps filter - after payloader
        pay_capsfilter = gst_element_factory_make("capsfilter", NULL);
        GstCaps* pay_caps = gst_caps_new_simple("application/x-rtp",
                                             "media", G_TYPE_STRING, "video",
                                             "encoding-name", G_TYPE_STRING, "H264",
                                             "payload", G_TYPE_INT, 96,
                                             NULL);

        g_object_set(G_OBJECT(pay_capsfilter), "caps", pay_caps, NULL);
        gst_caps_unref(pay_caps);

        // Create Nvidia FEC element
        rtpulpfecenc = gst_element_factory_make("rtpulpfecenc", NULL);

        // Add elements to pipeline
        gst_bin_add_many(GST_BIN(pipeline), src, src_capsfilter, src_queue, jpegdec_stage, conv, conv_capsfilter, 
                         enc_queue0, enc, enc_queue1, parse, pay, pay_capsfilter, rtpulpfecenc, NULL);
        
        // Link elements
        gst_element_link_many(src, src_capsfilter, src_queue, jpegdec_stage, conv, conv_capsfilter, 
                              enc_queue0, enc, enc_queue1, parse, pay, pay_capsfilter, rtpulpfecenc, NULL);

        // Link to webrtcbin
        GstPad* pay_src = gst_element_get_static_pad(rtpulpfecenc, "src");

        // Requesting sink pad from webrtcbin also creates a new transceiver internally
        GstPad* webrtc_sink = gst_element_get_request_pad(webrtcbin, "sink_%u");

        // Retrieve the transceiver associated with this pad
        GstWebRTCRTPTransceiver* transceiver = nullptr;
        g_signal_emit_by_name(webrtcbin, "get-transceiver", pipeline_stream_index, &transceiver);

        if (transceiver) {
            g_object_set(transceiver, "do-nack", TRUE, nullptr);
            
            // Store encoder element in the map
            encoders[pipeline_stream_index] = enc;

            RCLCPP_INFO(this->get_logger(), "Transceiver created for stream %d", pipeline_stream_index);

            // Create an instance of info
            StreamInfo* stream_info = new StreamInfo();

            // Store necessary data in the instance of stream info for further usage during
            // frame interval calculation and sending to receiver; dynamically adjusting bitrate and fps
            stream_info->stream_id = pipeline_stream_index;
            stream_info->encoder = enc;
            stream_info->videorate = videorate;

            // Attach stream_info to pad
            g_object_set_qdata_full(G_OBJECT(src_srcpad),
                STREAM_INFO_QUARK,
                stream_info,
                [](gpointer data){ delete static_cast<StreamInfo*>(data); });
            
            // Create frame callback probe
            gst_pad_add_probe(src_srcpad, GST_PAD_PROBE_TYPE_BUFFER, &WebRTCSendNode::frame_probe_callback, this, nullptr);
            gst_object_unref(src_srcpad);

            // Add stream_info instance to streams map
            {
                std::lock_guard<std::mutex> lk(streams_mutex);
                streams[pipeline_stream_index] = stream_info;
            }

            // Only increment if transceiver has actually been created
            pipeline_stream_index++;
            gst_object_unref(transceiver);
        } else {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not get transceiver for stream %d.", pipeline_stream_index);
        }

        gst_pad_link(pay_src, webrtc_sink);
        gst_object_unref(pay_src);
        gst_object_unref(webrtc_sink);
    }

    static GstPadProbeReturn frame_probe_callback(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
        WebRTCSendNode* self = static_cast<WebRTCSendNode*>(user_data);

        // Retrieve stream_info
        StreamInfo* stream_info = static_cast<StreamInfo*>(
            g_object_get_qdata(G_OBJECT(pad), STREAM_INFO_QUARK)
        );

        // Only process buffers
        if (!(GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER)) {
            return GST_PAD_PROBE_OK;
        }

        // Get current time point from system_clock (wall clock) in ns
        auto now = std::chrono::system_clock::now();
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

        // Lock the mutex to access stream_info
        std::lock_guard<std::mutex> lk(self->stream_info_struct_mutex);

        // Interval between frames' PTSs in nanoseconds
        uint64_t interval_ns = 0;
        
        if (stream_info->have_last_ts) {
            interval_ns = now_ns - stream_info->last_frame_ts_ns;
        } else {
            // First frame received, can't compute interval yet
            stream_info->last_frame_ts_ns = now_ns;
            stream_info->have_last_ts = true;

            // Don't process further for first frame
            return GST_PAD_PROBE_OK;
        }

        // Update last frame PTS
        stream_info->last_frame_ts_ns = now_ns;

        // If interval is non-positive, return
        if (interval_ns <= 0) return GST_PAD_PROBE_OK;

        if (stream_info->ewma_frame_interval_ns == 0) {
            stream_info->ewma_frame_interval_ns = interval_ns;
        } else {
            stream_info->ewma_frame_interval_ns =
                self->interval_ewma_alpha * interval_ns +
                (1.0 - self->interval_ewma_alpha) * stream_info->ewma_frame_interval_ns;
        }

        return GST_PAD_PROBE_OK;
    }

    void frame_interval_monitor_callback() {
        std::lock_guard<std::mutex> lk(streams_mutex);

        for (auto& [stream_id, stream_info] : streams) {
            if (!stream_info) continue;

            // Get EWMA frame interval
            uint64_t ewma_frame_interval_ns = 0;
            {
                std::lock_guard<std::mutex> lk(stream_info_struct_mutex);
                ewma_frame_interval_ns = stream_info->ewma_frame_interval_ns;
            }

            // Prepare frame interval message
            Json::Value info;
            bool send = false;

            info["info"]["type"] = "frame_interval";
            info["info"]["stream_id"] = stream_id;
            info["info"]["frame_interval_ns"] = Json::Value(Json::UInt64(ewma_frame_interval_ns));
            send = true;
            RCLCPP_WARN(this->get_logger(), "Stream %d: EWMA frame interval %d ns", stream_id, ewma_frame_interval_ns);

            if (send) {
                Json::StreamWriterBuilder w;
                std::string msg = Json::writeString(w, info);
                this->queue_ws(msg);
            }
        }
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
        // Create empty GStreamer pipeline
        GError* error = nullptr;
        pipeline = gst_pipeline_new("pipeline");
        if (!pipeline) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create GStreamer pipeline.");
            return;
        }

        // Create webrtcbin element
        webrtcbin = gst_element_factory_make("webrtcbin", "webrtcbin");
        if (!webrtcbin) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create webrtcbin element.");
            return;
        }

        // Set webrtcbin properties
        g_object_set(webrtcbin,
             "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE,
             "stun-server", "stun://stun.l.google.com:19302",
             NULL);
        
        // Add webrtcbin to pipeline
        gst_bin_add(GST_BIN(pipeline), webrtcbin);

        // Add media streams to the pipeline
        add_stream("argus", "", "video/x-raw", 640, 480, 30, 2000000);
        add_stream("v4l2src", "/dev/video11", "video/x-raw", 640, 480, 15, 2000000);
        add_stream("v4l2src", "/dev/cam-arducam", "video/x-raw", 640, 480, 30, 2000000);
        // add_stream("v4l2src", "/dev/cam-aveo", "video/x-raw", 640, 480, 30, 2000000);
        add_stream("v4l2src", "/dev/video9", "video/x-raw", 640, 480, 30, 2000000);

        gst_pipeline_use_clock(GST_PIPELINE(pipeline), gst_system_clock_obtain());

        // Connect to signals
        g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(&WebRTCSendNode::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCSendNode::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCSendNode::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    // void handle_receiver_stats(const Json::Value& stats) {
    //     // Get individual metrics and update member variables
    //     double l_jitter = stats.isMember("jitter") ? stats["jitter"].asDouble() : -1.0;
    //     int64_t l_packets_received = stats.isMember("packets_received") ? stats["packets_received"].asUInt64() : -1;
    //     int64_t l_packets_lost = stats.isMember("packets_lost") ? stats["packets_lost"].asUInt64() : -1;
    //     uint64_t l_bytes_received = stats.isMember("bytes_received") ? stats["bytes_received"].asUInt64() : -1;

    //     // Print or log stats
    //     RCLCPP_INFO(this->get_logger(),
    //         "Received Stats -> Jitter: %.4f | Bytes: %lu | Packets: %lu/%lu",
    //         l_jitter, l_bytes_received, l_packets_received, l_packets_lost
    //     );
    // }

    void change_bitrate(int stream_id, int delta_bps) {
        // Create a lambda that will execute the bitrate change
        auto func = [this, stream_id, delta_bps]() {
            StreamInfo* stream_info = nullptr;

            // Lock the map to find the StreamInfo
            {
                std::lock_guard<std::mutex> lk(streams_mutex);
                auto it = streams.find(stream_id);
                if (it == streams.end() || it->second == nullptr) {
                    RCLCPP_WARN(this->get_logger(),
                                "Stream %d not found. Cannot change bitrate.", stream_id);
                    return;
                }
                encoder = it->second->encoder;
            }

            if (!encoder) {
                RCLCPP_WARN(this->get_logger(),
                            "Stream %d encoder is null. Cannot change bitrate.", stream_id);
                return;
            }

            // // Find the encoder corresponding to stream_id
            // auto it = encoders.find(stream_id);
            // if (it == encoders.end() || it->second == nullptr) {
            //     RCLCPP_WARN(this->get_logger(),
            //                 "Stream %d encoder not found. Cannot change bitrate.", stream_id);
            //     return;
            // }
        
            // // Get the encoder element
            // GstElement* encoder = it->second;
            
            // Get the current bitrate
            int current_bitrate = 0;
            g_object_get(G_OBJECT(encoder), "bitrate", &current_bitrate, nullptr);

            // Calculate new bitrate
            int new_bitrate = current_bitrate + delta_bps;

            // Clamp new bitrate within allowed range
            new_bitrate = std::max(min_bitrate, std::min(new_bitrate, max_bitrate));

            // Set the new bitrate on the encoder
            g_object_set(G_OBJECT(encoder),
                     "bitrate",
                     static_cast<gint>(new_bitrate),
                     nullptr);

            RCLCPP_INFO(this->get_logger(),
                        "Changed stream %d bitrate to %d kbps",
                        stream_id, new_bitrate);

        };

        // Allocate lambda on heap for GLib callback
        auto* func_ptr = new std::function<void()>(func);

        g_main_context_invoke(nullptr, [](gpointer data) -> gboolean {
            auto* f = static_cast<std::function<void()>*>(data);
            (*f)();      // execute lambda
            delete f;    // free memory
            return G_SOURCE_REMOVE; // remove source after running once
        }, func_ptr);
    }

    /**
     * @brief Callback for sending ICE candidates
     * 
     * Called when a new ICE candidate is discovered
     */
    static void send_ice_candidate(GstElement* webrtcbin, guint mlineindex, gchar* candidate, gpointer user_data) {
        auto * self = static_cast<WebRTCSendNode*>(user_data);
        RCLCPP_INFO(self->get_logger(), "Sending ICE candidate: %s", candidate);

        // Create a JSON message to send over WebSocket
        Json::Value icemsg;
        icemsg["ice"]["candidate"] = candidate;
        icemsg["ice"]["sdpMLineIndex"] = static_cast<int>(mlineindex);  // Casting mlineindex to int for JSON

        // Convert JSON message to string
        Json::StreamWriterBuilder writer;
        std::string icemsg_str = Json::writeString(writer, icemsg);

        // Send the ICE candidate over WebSocket
        static_cast<WebRTCSendNode*>(user_data)->queue_ws(icemsg_str);
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
     * @brief Callback when WebRTC negotiation is needed
     */
    static void on_negotiation_needed(GstElement* webrtcbin, gpointer user_data) {
        auto * self = static_cast<WebRTCSendNode*>(user_data);
        RCLCPP_INFO(self->get_logger(), "Negotiation needed");

        GstPromise* promise = gst_promise_new_with_change_func(on_offer_created, user_data, nullptr);
        g_signal_emit_by_name(webrtcbin, "create-offer", nullptr, promise);
    }

    /**
     * @brief Callback when SDP offer is created
     */
    static void on_offer_created(GstPromise* promise, gpointer user_data) {
        auto * self = static_cast<WebRTCSendNode*>(user_data);
        GstWebRTCSessionDescription* offer = nullptr;
        const GstStructure* reply = gst_promise_get_reply(promise);
        gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
        gst_promise_unref(promise);

        if (!offer) {
            RCLCPP_ERROR(self->get_logger(), "Failed to create offer");
            return;
        }

        gchar* sdp_str = gst_sdp_message_as_text(offer->sdp);
        RCLCPP_INFO(self->get_logger(), "Created offer: %s", sdp_str);

        Json::Value sdpmsg;
        sdpmsg["sdp"]["type"] = "offer";
        sdpmsg["sdp"]["sdp"] = sdp_str;

        Json::StreamWriterBuilder writer;
        std::string sdpmsg_str = Json::writeString(writer, sdpmsg);

        static_cast<WebRTCSendNode*>(user_data)->queue_ws(sdpmsg_str);

        g_signal_emit_by_name(static_cast<WebRTCSendNode*>(user_data)->webrtcbin, "set-local-description", offer, nullptr);

        g_free(sdp_str);
        gst_webrtc_session_description_free(offer);
    }

    /**
     * @brief Handles incoming SDP messages
     * 
     * @param sdp JSON string containing SDP information
     */
    void handle_sdp_answer(const std::string& sdp) {
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(sdp);

        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            std::string sdpType = jsonMsg["sdp"]["type"].asString();
            std::string sdp_str = jsonMsg["sdp"]["sdp"].asString();

            if (sdpType == "answer") {
                GstSDPMessage* sdp_msg = nullptr;
                gst_sdp_message_new(&sdp_msg);
                GstSDPResult result = gst_sdp_message_parse_buffer((guint8*)sdp_str.c_str(), sdp_str.size(), sdp_msg);

                if (result != GST_SDP_OK) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse SDP message: %d", result);
                    return;
                }

                GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
                if (!answer) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create WebRTC session description");
                    gst_sdp_message_free(sdp_msg);
                    return;
                }

                g_signal_emit_by_name(webrtcbin, "set-remote-description", answer, nullptr);
                gst_webrtc_session_description_free(answer);

            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported SDP type: %s", sdpType.c_str());
            }
        
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse SDP answer: %s", errs.c_str());
        }
    }

    /**
     * @brief Callback for handling incoming media streams
     */
    static void on_incoming_stream(GstElement* webrtcbin, GstPad* pad, WebRTCSendNode* self) {
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
     * Sets up appropriate elements for handling decoded audio/video streams
     */
    static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, WebRTCSendNode* self) {
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

            g_object_set(G_OBJECT(sink), "sync", FALSE, nullptr);

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
    std::thread ws_thread;               // Thread for WebSocket message processing
    GstElement* pipeline;                // Main GStreamer pipeline
    GstElement* webrtcbin;              // WebRTC element
    websocketpp::connection_hdl global_hdl;  // WebSocket connection handle
    websocketpp::client<websocketpp::config::asio_client>* global_client;  // WebSocket client

    std::queue<std::string> msg_queue;   // Queue for outgoing WebSocket messages
    std::mutex ws_mutex;                 // Mutex for thread safety
    std::condition_variable ws_cv;       // Condition variable for message queue

    // Map of encoder elements of each stream
    std::unordered_map<int, GstElement*> encoders;

    // Index, used only during pipeline creation,
    // to correctly map encoder element to its corresponding stream
    int pipeline_stream_index = 0;

    // Bitrate
    const int min_bitrate{300000};
    const int max_bitrate{2000000};

    struct StreamInfo {
        int stream_id;
        bool have_last_ts{false};
        uint64_t last_frame_ts_ns{0};
        uint64_t ewma_frame_interval_ns{0}; // Exponentially Weighted Moving Average
        GstElement* encoder{nullptr};
        GstElement* videorate{nullptr};
    };

    std::mutex stream_info_struct_mutex;
    rclcpp::TimerBase::SharedPtr frame_interval_monitor_timer;
    const double interval_ewma_alpha = 0.1;

    // Map of stream info instances
    std::unordered_map<int, StreamInfo*> streams;
    std::mutex streams_mutex;

    // // Vector of frame interval info instances
    // std::vector<StreamInfo*> frame_intervals;
    // std::mutex frame_intervals_vector_mutex;
    const int frame_interval_monitor_period_ms = 10000;
};

/**
 * @brief Main entry point
 * 
 * Initializes X11 threading, creates WebRTCSendNode instance,
 * and runs the main loop
 */
int main(int argc, char* argv[]) {
    if (!XInitThreads()) {
        RCLCPP_FATAL(rclcpp::get_logger("WebRTCSendNode"), "Failed to initialize X11 threading!");
        return 1;
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebRTCSendNode>();

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
    node->ws_running = false;
    ws_connect_thread.join();
    rclcpp::shutdown();
    return 0;
}