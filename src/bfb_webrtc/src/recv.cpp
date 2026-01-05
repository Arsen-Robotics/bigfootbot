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
#include <gst/rtp/gstrtpbuffer.h>

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

        // Stutter monitor callback
        // stutter_monitor_timer = this->create_wall_timer(
        //     std::chrono::milliseconds(stutter_monitor_period_ms),
        //     [this]() { this->stutter_monitor_callback(); }
        // );
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

            } else if (jsonMsg.isMember("info")) {
                const auto& info = jsonMsg["info"];
                std::string type = info["type"].asString();

                if (type == "frame_interval") {
                    int stream_id = info["stream_id"].asInt();
                    uint64_t frame_interval_ns = info["frame_interval_ns"].asUInt64();

                    RCLCPP_INFO(this->get_logger(), "Stream %d frame_interval %lu", stream_id, frame_interval_ns);

                    // Lock the streams map
                    std::lock_guard<std::mutex> lk(streams_mutex);

                    auto it = streams.find(stream_id);
                    if (it != streams.end() && it->second != nullptr) {
                        StreamInfo* stream_info = it->second;

                        // Lock the stream_info instance
                        std::lock_guard<std::mutex> lk1(stutter_mutex);

                        // Update the frame interval (nominal)
                        stream_info->nominal_frame_interval_ns = frame_interval_ns;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Couldn't find stream ID %d in streams map", stream_id);
                    }
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

        // Grab rtpbin *by name* from webrtcbin’s internal bin
        GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(webrtcbin), "rtpbin");
        if (rtpbin) {
            g_signal_connect(rtpbin,
                            "new-jitterbuffer",
                            G_CALLBACK(on_new_jitterbuffer),
                            this);
            gst_object_unref(rtpbin);
        }

        gst_pipeline_use_clock(GST_PIPELINE(pipeline), gst_system_clock_obtain());

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCRecvNode::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCRecvNode::on_incoming_stream), this);

        // Set pipeline state to PAUSED
        gst_element_set_state(pipeline, GST_STATE_PAUSED);
    }

    void stutter_monitor_callback() {
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
                                            stream_id, ratio, stutters, frames);

            int delta = 0;

            if (ratio > 0.15) delta = -500000;   // -500 kbps
            else if (ratio > 0.11) delta = -300000; // -300 kbps
            else if (ratio > 0.07) delta = -100000; // -100 kbps
            else if (ratio > 0.06) delta = -50000; // -50 kbps

            else if (ratio < 0.04) delta = +80000;  // +80 kbps
            else if (ratio < 0.015) delta = +150000; // +150 kbps (very clean)

            if (delta != 0) {
                // Prepare control message
                Json::Value ctrl;

                ctrl["control"]["action"] = "change_bitrate";
                ctrl["control"]["stream_id"] = stream_id;
                ctrl["control"]["delta"] = delta;

                Json::StreamWriterBuilder w;
                std::string msg = Json::writeString(w, ctrl);
                //this->queue_ws(msg);
            }

            // if (ratio > high_stutter_threshold) {
            //     // Too much stutter -> request lower bitrate
            //     int delta = -bitrate_step_kbps;
            //     ctrl["control"]["action"] = "change_bitrate";
            //     ctrl["control"]["stream_id"] = stream_id;
            //     ctrl["control"]["delta"] = delta;
            //     send = true;
            //     RCLCPP_WARN(this->get_logger(), "Stream %d: High stutter ratio %.3f -> request bitrate delta %d kbps", stream_id, ratio, delta);
            // } else if (ratio < low_stutter_threshold) {
            //     // Low stutter -> request higher bitrate
            //     int delta = bitrate_step_kbps;
            //     ctrl["control"]["action"] = "change_bitrate";
            //     ctrl["control"]["stream_id"] = stream_id;
            //     ctrl["control"]["delta"] = delta;
            //     send = true;
            //     RCLCPP_WARN(this->get_logger(), "Stream %d: Low stutter ratio %.3f -> request bitrate delta %d kbps", stream_id, ratio, delta);
            // }

            // if (send) {
            //     Json::StreamWriterBuilder w;
            //     std::string msg = Json::writeString(w, ctrl);
            //     this->queue_ws(msg);
            // }
        }
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

                // this->configure_jitterbuffer(this->webrtcbin);

                // Set pipeline state to PLAYING
                gst_element_set_state(this->pipeline, GST_STATE_PLAYING);
                g_timeout_add_seconds(1, print_webrtc_stats, this);

                g_timeout_add_seconds(1, [](gpointer data) -> gboolean {
                    auto* self = static_cast<WebRTCRecvNode*>(data);
                    self->analyze_packet_loss();
                    return TRUE; // keep running
                }, this);

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

    static void
    on_new_jitterbuffer(GstElement *rtpbin,
                        GstElement *jitterbuffer,
                        guint session,
                        guint ssrc,
                        gpointer user_data)
    {
        // Basic config — adapt as needed
        g_object_set(jitterbuffer,
                    "mode", 1,                      // SLAVE mode (default, best for live)
                    "latency", 500,                 // 300ms - LARGE buffer for maximum smoothness
                    "do-lost", FALSE,               // Don't signal lost packets
                    "do-retransmission", FALSE,      // Enable NACK retransmissions
                    "drop-on-latency", FALSE,       // NEVER drop late frames
                    "max-dropout-time", 5000,       // 5s before considering stream dead
                    "max-misorder-time", 2000,       // 200ms tolerance for reordering
                    nullptr);

        g_printerr("Configured rtpjitterbuffer for session %u, ssrc %u: %s\n",
                session, ssrc, GST_ELEMENT_NAME(jitterbuffer));

        auto* self = static_cast<WebRTCRecvNode*>(user_data);

        // PRE-JB (network arrival)
        GstPad* sinkpad = gst_element_get_static_pad(jitterbuffer, "sink");
        gst_pad_add_probe(
            sinkpad,
            GST_PAD_PROBE_TYPE_BUFFER,
            pre_jb_probe,
            self,
            nullptr
        );
        gst_object_unref(sinkpad);

        // POST-JB (after reordering / dropping)
        GstPad* srcpad = gst_element_get_static_pad(jitterbuffer, "src");
        gst_pad_add_probe(
            srcpad,
            GST_PAD_PROBE_TYPE_BUFFER,
            post_jb_probe,
            self,
            nullptr
        );
        gst_object_unref(srcpad);
    }

    static GstPadProbeReturn pre_jb_probe(
        GstPad*, GstPadProbeInfo* info, gpointer user_data)
    {
        auto* self = static_cast<WebRTCRecvNode*>(user_data);
        GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buf) return GST_PAD_PROBE_OK;

        GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
        if (!gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp))
            return GST_PAD_PROBE_OK;

        uint16_t seq = gst_rtp_buffer_get_seq(&rtp);
        uint32_t ssrc = gst_rtp_buffer_get_ssrc(&rtp);

        auto& tr = self->ssrc_map[ssrc];

        tr.pre_jb_seen.insert(seq);

        if (!tr.initialized) {
            tr.max_pre_seq = seq;
            tr.initialized = true;
            tr.last_check = std::chrono::steady_clock::now();
        } else if (seq_less(tr.max_pre_seq, seq)) {
            tr.max_pre_seq = seq;
        }

        gst_rtp_buffer_unmap(&rtp);
        return GST_PAD_PROBE_OK;
    }

    static GstPadProbeReturn post_jb_probe(
        GstPad*, GstPadProbeInfo* info, gpointer user_data)
    {
        auto* self = static_cast<WebRTCRecvNode*>(user_data);
        GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buf) return GST_PAD_PROBE_OK;

        GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
        if (!gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp))
            return GST_PAD_PROBE_OK;

        uint16_t seq = gst_rtp_buffer_get_seq(&rtp);
        uint32_t ssrc = gst_rtp_buffer_get_ssrc(&rtp);

        auto& tr = self->ssrc_map[ssrc];
        tr.post_jb_seen.insert(seq);

        if (seq_less(tr.max_post_seq, seq))
            tr.max_post_seq = seq;

        gst_rtp_buffer_unmap(&rtp);
        return GST_PAD_PROBE_OK;
    }

    void analyze_packet_loss()
    {
        auto now = std::chrono::steady_clock::now();

        for (auto& [ssrc, tr] : ssrc_map) {
            if (now - tr.last_check < std::chrono::seconds(1))
                continue;

            tr.last_check = now;

            uint16_t upper = tr.max_pre_seq;

            for (uint16_t seq = 0; seq != upper; ++seq) {

                bool seen_pre  = tr.pre_jb_seen.count(seq);
                bool seen_post = tr.post_jb_seen.count(seq);

                if (!seen_pre) {
                    std::cout << "[NETWORK DROP] SSRC=" << ssrc
                            << " seq=" << seq << "\n";
                }
                else if (!seen_post) {
                    std::cout << "[JB DROP] SSRC=" << ssrc
                            << " seq=" << seq << "\n";
                }
            }

            // Cleanup old history (keep sliding window)
            const uint16_t KEEP = 5000;
            while (tr.pre_jb_seen.size() > KEEP)
                tr.pre_jb_seen.erase(tr.pre_jb_seen.begin());

            while (tr.post_jb_seen.size() > KEEP)
                tr.post_jb_seen.erase(tr.post_jb_seen.begin());
        }
    }

    static inline bool seq_less(uint16_t a, uint16_t b) {
        return (int16_t)(a - b) < 0;
    }

    // static void configure_jitterbuffer(GstElement* webrtcbin) {
    //     // Iterate through all children to find rtpjitterbuffer elements
    //     GstIterator* it = gst_bin_iterate_elements(GST_BIN(webrtcbin));
    //     GValue item = G_VALUE_INIT;
        
    //     while (gst_iterator_next(it, &item) == GST_ITERATOR_OK) {
    //         GstElement* element = GST_ELEMENT(g_value_get_object(&item));
    //         const gchar* factory_name = gst_plugin_feature_get_name(
    //             GST_PLUGIN_FEATURE(gst_element_get_factory(element))
    //         );
            
    //         if (g_strcmp0(factory_name, "rtpjitterbuffer") == 0) {
    //             // CRITICAL: Disable lost packet events
    //             g_object_set(element,
    //                 "mode", 4,                      // SLAVE mode (sync to sender clock)
    //                 "latency", 220,                 // 220ms (match Chrome's ~217ms)
    //                 "do-lost", FALSE,               // Don't generate lost events
    //                 "do-retransmission", TRUE,      // Enable NACK retransmission
    //                 "rtx-max-retries", 3,           // Allow 3 retransmit attempts
    //                 "drop-on-latency", FALSE,       // Don't drop frames
    //                 "max-dropout-time", 500,
    //                 "max-misorder-time", 200,
    //                 NULL);
                
    //             g_print("Configured jitterbuffer: %s\n", GST_ELEMENT_NAME(element));
    //         }
            
    //         g_value_reset(&item);
    //     }
        
    //     g_value_unset(&item);
    //     gst_iterator_free(it);
    // }

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

        //g_object_set(self->webrtcbin, "latency", 100, NULL);

        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);

        // Create elements
        GstElement* conv = nullptr;
        GstElement* queue = nullptr;
        GstElement* identity = nullptr;
        GstElement* sink = nullptr;

        uint64_t nominal_frame_interval_ns = 33333333ULL; // Default to ~30 FPS

        if (g_str_has_prefix(name, "video")) {
            // Get nominal framerate from caps and calculate frame interval in ns
            gint fps_n = 0, fps_d = 1;
            if (gst_structure_get_fraction(str, "framerate", &fps_n, &fps_d) && fps_d != 0) {
                nominal_frame_interval_ns = static_cast<uint64_t>((1e9 * fps_d / fps_n) + 0.5);
            }
            
            conv = gst_element_factory_make("videoconvert", nullptr);
            g_object_set(conv,
                "qos", FALSE, // Enable QoS
                // "n-threads", 2, // Use multiple threads for conversion
                NULL);

            queue = gst_element_factory_make("queue", nullptr);
            g_object_set(queue,
                "max-size-buffers", 33,
                "leaky", 2, // downstream
                NULL);

            identity = gst_element_factory_make("identity", nullptr);
            g_object_set(identity,
                "single-segment", TRUE,
                nullptr);

            // int stream_id = stream_info->stream_id;
            // std::string filename = "/ros2_ws/src/stream" +
            // std::to_string(stream_id) + ".mp4";
            
            sink = gst_element_factory_make("ximagesink", nullptr);
            g_object_set(sink,
                "sync", FALSE,           // Match Chrome's synchronized playback
                "max-lateness", 2000000000, // 200ms tolerance
                "qos", FALSE,
                "async", TRUE,
                NULL);

            
        } else if (g_str_has_prefix(name, "audio")) {
            conv = gst_element_factory_make("audioconvert", nullptr);
            sink = gst_element_factory_make("autoaudiosink", nullptr);
        }

        gst_bin_add_many(GST_BIN(self->pipeline), conv, queue, identity, sink, nullptr);
        gst_element_sync_state_with_parent(conv);
        gst_element_sync_state_with_parent(queue);
        gst_element_sync_state_with_parent(identity);
        gst_element_sync_state_with_parent(sink);

        // Link: decodebin pad → conv → queue → identity → sink
        GstPad* sinkpad = gst_element_get_static_pad(conv, "sink");
        gst_pad_link(pad, sinkpad);
        gst_object_unref(sinkpad);

        gst_element_link_many(conv, queue, identity, sink, nullptr);

        // Add stutter monitoring probe on sink pad of sink element
        // if (sink) {
        //     GstPad* sinkpad = gst_element_get_static_pad(sink, "sink");
        //     if (sinkpad) {
        //         // Store nominal frame interval on pad for probe use
        //         stream_info->nominal_frame_interval_ns = nominal_frame_interval_ns;

        //         // Attach stream info to pad
        //         // Transfer ownership to sinkpad: sinkpad will free the struct when pad is destroyed
        //         g_object_set_qdata_full(G_OBJECT(sinkpad),
        //             STREAM_INFO_QUARK,
        //             stream_info,
        //             [](gpointer data){ delete static_cast<StreamInfo*>(data); });

        //         // Remove the non-owning pointer from decodebin to avoid dangling pointer later
        //         g_object_set_qdata(G_OBJECT(decodebin), STREAM_INFO_QUARK, nullptr);

        //         // Add probe
        //         gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, &WebRTCRecvNode::frame_probe_callback, self, nullptr);
        //         gst_object_unref(sinkpad);
        //     }
        // }

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

    static gboolean print_webrtc_stats(gpointer user_data) {
        auto* self = static_cast<WebRTCRecvNode*>(user_data);

        if (!self->webrtcbin)
            return G_SOURCE_CONTINUE;

        GstPromise* promise = gst_promise_new();

        // Trigger stats collection
        g_signal_emit_by_name(self->webrtcbin, "get-stats", nullptr, promise);

        gst_promise_wait(promise);

        const GstStructure* stats = gst_promise_get_reply(promise);

        if (!stats) {
            gst_promise_unref(promise);
            return G_SOURCE_CONTINUE;
        }

        // Iterate all stats entries
        gst_structure_foreach(stats,
            [](GQuark field_id, const GValue* value, gpointer) -> gboolean {

                if (!GST_VALUE_HOLDS_STRUCTURE(value))
                    return TRUE;

                const GstStructure* s = gst_value_get_structure(value);
                const gchar* type = gst_structure_get_name(s);

                // Only inbound RTP video
                if (g_str_has_prefix(type, "inbound-rtp")) {
                    guint64 packets_lost = 0;
                    guint64 packets_recv = 0;

                    gst_structure_get_uint64(s, "packets-lost", &packets_lost);
                    gst_structure_get_uint64(s, "packets-received", &packets_recv);

                    g_print("[STATS] %s recv=%" G_GUINT64_FORMAT
                            " lost=%" G_GUINT64_FORMAT "\n",
                            type, packets_recv, packets_lost);
                }

                return TRUE;
            },
            nullptr
        );

        gst_promise_unref(promise);
        return G_SOURCE_CONTINUE; // keep ticking
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
    std::mutex stutter_mutex;
    rclcpp::TimerBase::SharedPtr stutter_monitor_timer;

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
    const double stutter_multiplier = 1.1; // multiplier for PTS interval threshold
    const int stutter_monitor_period_ms = 3000;    // monitor period in ms
    //const double high_stutter_threshold = 0.07; // if >7% stutter -> reduce bitrate
    //const double low_stutter_threshold = 0.04;  // if <1% stutter -> increase bitrate
    //const int bitrate_step_kbps = 100000;     // amount to change bitrate by (bps)

    struct SeqTracker {
        std::set<uint16_t> pre_jb_seen;     // packets seen BEFORE JB
        std::set<uint16_t> post_jb_seen;    // packets seen AFTER JB

        uint16_t max_pre_seq = 0;
        uint16_t max_post_seq = 0;

        bool initialized = false;

        std::chrono::steady_clock::time_point last_check;
    };

    std::unordered_map<uint32_t, SeqTracker> ssrc_map;
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
    // std::thread gloop_thread([&]() { 
    //     g_main_loop_run(loop); 
    // });

    // Start WebSocket connect in separate thread
    std::thread ws_connect_thread([&]() { 
        node->connect(); 
    });

    // ROS2 spin in separate thread
    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    // Main thread
    g_main_loop_run(loop);

    // Cleanup
    node->ws_running = false;
    ws_connect_thread.join();
    rclcpp::shutdown();
    return 0;
}