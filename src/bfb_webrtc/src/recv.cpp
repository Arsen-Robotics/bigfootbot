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
#include <sys/resource.h>

static GQuark STREAM_INFO_QUARK = 0;

/**
 * @brief Main class handling WebRTC video streaming
 * 
 * This class manages the WebRTC connection, GStreamer pipeline,
 * and WebSocket signaling to stream video from multiple cameras
 */
class WebRTCRecvNode : public rclcpp::Node {
public:
    // Flag to control WebSocket thread
    std::atomic<bool> ws_running{true};

    /**
     * @brief Constructor - initializes GStreamer and member variables
     */
    WebRTCRecvNode() : Node("webrtc_recv_node") {
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
    }

    /**
     * @brief Destructor - cleans up resources
     */
    ~WebRTCRecvNode() {
        // Stop WebSocket thread
        ws_running = false;
        ws_cv.notify_all();

        if (ws_thread.joinable()) {
            ws_thread.join();
        }

        // Stop stutter monitor thread
        stutter_monitor_running = false;
        if (stutter_monitor_thread.joinable()) {
            stutter_monitor_thread.join();
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

            // Disable WebSocket++ logs
            client.clear_access_channels(websocketpp::log::alevel::all);
            
            // Optionally, keep only connection logs (if you still want to know when connected/disconnected):
            client.set_access_channels(websocketpp::log::alevel::connect | websocketpp::log::alevel::disconnect);

            client.set_open_handler(std::bind(&WebRTCRecvNode::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCRecvNode::on_msg, this, std::placeholders::_1, std::placeholders::_2));

            websocketpp::lib::error_code ec;
            websocketpp::client<websocketpp::config::asio_client>::connection_ptr con = client.get_connection("ws://87.119.173.184:8765", ec);

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
                auto sdp_func = [this, payload_copy]() { this->handle_sdp_offer(payload_copy); };
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

    /**
     * @brief Queues a message to be sent over WebSocket
     * 
     * @param msg Message to be queued
     */
    void queue_ws(const std::string& msg) {
        std::lock_guard<std::mutex> lk(ws_mutex);
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
        pipeline = gst_parse_launch("webrtcbin name=webrtcbin stun-server=stun://stun.l.google.com:19302 \
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
        webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "webrtcbin");
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

        // Start stutter monitor thread (runs until destructor)
        stutter_monitor_running = true;
        stutter_monitor_thread = std::thread([this]() {
            while (stutter_monitor_running && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(stutter_monitor_period_ms));

                std::lock_guard<std::mutex> lk(streams_mutex);

                for (auto& [stream_id, stream_info] : streams) {
                    if (!stream_info) continue;

                    // Get current stutter metrics and reset counters
                    uint64_t frames = 0;
                    uint64_t stutters = 0;
                    {
                        std::lock_guard<std::mutex> lk(stutter_mutex);
                        frames = stream_info->frame_count;
                        stutters = stream_info->stutter_count;
                        stream_info->frame_count = 0; // Reset for next period
                        stream_info->stutter_count = 0; // Reset for next period
                    }

                    if (frames == 0) continue; // Avoid division by zero

                    // Calculate stutter ratio
                    double ratio = double(stutters) / double(frames);

                    RCLCPP_INFO(this->get_logger(), "Stutter ratio for stream %d: %.3f (%llu stutters / %llu frames)",
                                                    stream_info->stream_id, ratio, stutters, frames);

                    // Prepare control message
                    Json::Value ctrl;
                    bool send = false;
                    if (ratio > high_stutter_threshold) {
                        // Too much stutter -> request lower bitrate
                        int delta = -bitrate_step_kbps;
                        ctrl["control"]["action"] = "change_bitrate";
                        ctrl["control"]["stream_id"] = stream_info->stream_id;
                        ctrl["control"]["delta"] = delta;
                        send = true;
                        RCLCPP_WARN(this->get_logger(), "Stream %d: High stutter ratio %.3f -> request bitrate delta %d kbps", stream_info->stream_id, ratio, delta);
                    } else if (ratio < low_stutter_threshold) {
                        // Low stutter -> request higher bitrate
                        int delta = bitrate_step_kbps;
                        ctrl["control"]["action"] = "change_bitrate";
                        ctrl["control"]["stream_id"] = stream_info->stream_id;
                        ctrl["control"]["delta"] = delta;
                        send = true;
                        RCLCPP_WARN(this->get_logger(), "Stream %d: High stutter ratio %.3f -> request bitrate delta %d kbps", stream_info->stream_id, ratio, delta);
                    }

                    if (send) {
                        Json::StreamWriterBuilder w;
                        std::string msg = Json::writeString(w, ctrl);
                        this->queue_ws(msg);
                    }
                }
            }
        });
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
                    WebRTCRecvNode* self = static_cast<WebRTCRecvNode*>(user_data);
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

        // Pad name is "src_N" where N is the transceiver index
        const gchar* pad_name = gst_pad_get_name(pad);
        RCLCPP_INFO(self->get_logger(), "Pad name: %s", pad_name);
        
        int transceiver_index;
        if (sscanf(pad_name, "src_%d", &transceiver_index) == 1) {
            RCLCPP_INFO(self->get_logger(), "Transceiver index: %d", transceiver_index);
            
            // Store transceiver index (stream_id) on pad
            StreamInfo* stream_info = new StreamInfo();
            stream_info->stream_id = transceiver_index;

            // Attach stream info to pad
            g_object_set_qdata(G_OBJECT(decodebin),
                STREAM_INFO_QUARK,
                stream_info);

            // Add stream_info instance to stream map
            {
                std::lock_guard<std::mutex> lk(self->streams_mutex);
                self->streams[stream_info->stream_id] = stream_info;
            }
        } else {
            RCLCPP_ERROR(self->get_logger(), "Couldn't get transceiver index, skipping stream");
            return;
        }

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
        GstCaps* caps = gst_pad_get_current_caps(pad);
        if (!caps) return;

        // Retrieve StreamInfo attached to decodebin
        StreamInfo* stream_info = static_cast<StreamInfo*>(
            g_object_get_qdata(G_OBJECT(decodebin), STREAM_INFO_QUARK)
        );

        if (!stream_info) {
            RCLCPP_ERROR(self->get_logger(), "No stream-info found for decodebin, skipping stream");
            return;
        }

        g_object_set(self->webrtcbin, "latency", 0, NULL);

        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);

        // Create elements
        GstElement* queue = nullptr;
        GstElement* conv = nullptr;
        GstElement* sink = nullptr;

        uint64_t nominal_frame_interval_ns = 33333333ULL; // Default to ~30 FPS

        if (g_str_has_prefix(name, "video")) {
            // Get nominal framerate from caps and calculate frame interval in ns
            gint fps_n = 0, fps_d = 1;
            if (gst_structure_get_fraction(str, "framerate", &fps_n, &fps_d) && fps_d != 0) {
                nominal_frame_interval_ns = static_cast<uint64_t>((1e9 * fps_d / fps_n) + 0.5);
            }
            
            // Create queue to help absorb jitter
            queue = gst_element_factory_make("queue", nullptr);
            g_object_set(queue,
                "max-size-buffers", 20,
                "max-size-time", G_GUINT64_CONSTANT(0),
                "max-size-bytes", 0,
                "leaky", 2, // downstream
                "silent", TRUE,
                "flush-on-eos", TRUE,
                NULL);

            conv = gst_element_factory_make("videoconvert", nullptr);
            g_object_set(conv,
                "qos", TRUE, // Enable QoS
                // "n-threads", 2, // Use multiple threads for conversion
                NULL);
            
            sink = gst_element_factory_make("xvimagesink", nullptr);
            g_object_set(sink,
                "sync", FALSE,
                "max-lateness", nominal_frame_interval_ns, // 1 frame interval
                "qos", TRUE, // Enable QoS
                NULL);

            
        } else if (g_str_has_prefix(name, "audio")) {
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        }

        gst_bin_add_many(GST_BIN(self->pipeline), queue, conv, sink, nullptr);
        gst_element_sync_state_with_parent(queue);
        gst_element_sync_state_with_parent(conv);
        gst_element_sync_state_with_parent(sink);

        // Link: decodebin pad → queue → conv → sink
        GstPad* sinkpad = gst_element_get_static_pad(queue, "sink");
        gst_pad_link(pad, sinkpad);
        gst_object_unref(sinkpad);

        gst_element_link_many(queue, conv, sink, nullptr);

        // Add stutter monitoring probe on sink pad of sink element
        if (sink) {
            GstPad* sinkpad = gst_element_get_static_pad(sink, "sink");
            if (sinkpad) {
                // Store nominal frame interval on pad for probe use
                stream_info->nominal_frame_interval_ns = nominal_frame_interval_ns;

                // Attach stream info to pad
                // Transfer ownership to sinkpad: sinkpad will free the struct when pad is destroyed
                g_object_set_qdata_full(G_OBJECT(sinkpad),
                    STREAM_INFO_QUARK,
                    stream_info,
                    [](gpointer data){ delete static_cast<StreamInfo*>(data); });

                // Remove the non-owning pointer from decodebin to avoid dangling pointer later
                g_object_set_qdata(G_OBJECT(decodebin), STREAM_INFO_QUARK, nullptr);

                // Add probe
                gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, &WebRTCRecvNode::frame_probe_callback, self, nullptr);
                gst_object_unref(sinkpad);
            }
        }

        gst_caps_unref(caps);
    }

    static GstPadProbeReturn frame_probe_callback(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
        WebRTCRecvNode* self = static_cast<WebRTCRecvNode*>(user_data);

        // Retrieve stream info
        StreamInfo* stream_info = static_cast<StreamInfo*>(
            g_object_get_qdata(G_OBJECT(pad), STREAM_INFO_QUARK)
        );

        // If this probe info doesn't contain a buffer, return
        if (!(GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER)) {
            return GST_PAD_PROBE_OK;
        }

        // Get the buffer (frame)
        GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buffer) return GST_PAD_PROBE_OK;
        
        // Retrieve PTS from buffer
        if (!GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))) {
            return GST_PAD_PROBE_OK;
        }

        // Get PTS in nanoseconds
        uint64_t pts_ns = GST_BUFFER_PTS(buffer);

        // Interval between frames' PTSs in nanoseconds
        uint64_t interval_ns = 0;

        {
            std::lock_guard<std::mutex> lk(self->stutter_mutex);
            
            if (stream_info->have_last_pts) {
                interval_ns = pts_ns - stream_info->last_frame_pts_ns;
            } else {
                // First frame received, can't compute interval yet
                stream_info->last_frame_pts_ns = pts_ns;
                stream_info->have_last_pts = true;

                // Don't process further for first frame
                return GST_PAD_PROBE_OK;
            }

            // Update last frame PTS
            stream_info->last_frame_pts_ns = pts_ns;

            // If interval is non-positive, return
            if (interval_ns <= 0) return GST_PAD_PROBE_OK;

            // Count frame and check for stutter
            stream_info->frame_count++;

            // Calculate PTS interval threshold beyond which a stutter is counted
            uint64_t threshold_ns = stream_info->nominal_frame_interval_ns * self->stutter_multiplier;

            if (interval_ns > threshold_ns) {
                stream_info->stutter_count++;
            }

            return GST_PAD_PROBE_OK;
        }
    }

private:
    // GStreamer elements
    GstElement* pipeline;
    GstElement* webrtcbin;

    // WebSocket variables
    websocketpp::connection_hdl global_hdl;
    websocketpp::client<websocketpp::config::asio_client>* global_client;
    std::thread ws_thread;
    std::queue<std::string> msg_queue;
    std::mutex ws_mutex;
    std::condition_variable ws_cv;

    // Stutter monitoring
    std::atomic<bool> stutter_monitor_running{false};
    std::thread stutter_monitor_thread;
    std::mutex stutter_mutex;

    // Stream info (per stream)
    struct StreamInfo {
        int stream_id;
        uint64_t nominal_frame_interval_ns{33333333ULL}; // Default to ~30 FPS
        bool have_last_pts{false};
        uint64_t last_frame_pts_ns{0};
        std::atomic<uint64_t> frame_count{0};
        std::atomic<uint64_t> stutter_count{0};
    };

    // Map of stream info instances
    std::unordered_map<int, StreamInfo*> streams;
    std::mutex streams_mutex;

    // Stutter detection parameters
    const double ema_alpha = 0.4;         // smoothing factor for EMA
    const double stutter_multiplier = 1.1; // multiplier for PTS interval threshold
    const int stutter_monitor_period_ms = 2000;    // monitor period in ms
    const double high_stutter_threshold = 0.07; // if >7% stutter -> reduce bitrate
    const double low_stutter_threshold = 0.01;  // if <1% stutter -> increase bitrate
    const int bitrate_step_kbps = 100;     // amount to change bitrate by (kbps)
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
    auto node = std::make_shared<WebRTCRecvNode>();

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