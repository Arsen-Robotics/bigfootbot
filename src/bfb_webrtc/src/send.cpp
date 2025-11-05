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
    std::atomic<bool> bitrate_running{false};

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
        bitrate_running = false;
        if (bitrate_thread.joinable()) {
            bitrate_thread.join();
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
                if (action == "set_bitrate") {
                    int bitrate = ctrl["value_kbps"].asInt();

                    change_bitrate(bitrate * 1000); // Convert kbps to bps
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

    void add_stream(int stream_id,
                    const std::string& device,
                    const std::string& src_type,
                    int width,
                    int height,
                    int framerate,
                    int bitrate) {
        // Create GStreamer elements for the stream
        GstElement *src, *capsfilter0, *convert, *capsfilter1 *queue0, *encoder, *queue1, *parse, *payloader, *capsfilter2;

        // Source
        if (src_type == "v4l2src") {
            src = gst_element_factory_make("v4l2src", NULL);
            g_object_set(G_OBJECT(src), "device", device.c_str(), NULL);
        } else if (src_type == "argus") {
            src = gst_element_factory_make("nvarguscamerasrc", NULL);
            g_object_set(G_OBJECT(src), "sensor-mode", 3, NULL);
        }

        // 1st caps filter - after camera source
        capsfilter0 = gst_element_factory_make("capsfilter", NULL);
        if (src_type == "v4l2src") {
            GstCaps* caps0 = gst_caps_new_simple("video/x-raw",
                                                "width", G_TYPE_INT, width,
                                                "height", G_TYPE_INT, height,
                                                "framerate", GST_TYPE_FRACTION, framerate, 1,
                                                NULL);
            g_object_set(G_OBJECT(capsfilter0), "caps", caps0, NULL);
            gst_caps_unref(caps0);
        } else if (src_type == "argus") {
            GstCaps* caps0 = gst_caps_new_simple("video/x-raw(memory:NVMM)",
                                                "width", G_TYPE_INT, width,
                                                "height", G_TYPE_INT, height,
                                                "framerate", GST_TYPE_FRACTION, framerate, 1,
                                                NULL);
            g_object_set(G_OBJECT(capsfilter0), "caps", caps0, NULL);
            gst_caps_unref(caps0);
        }
        
        // Converter
        convert = gst_element_factory_make("nvvidconv", NULL);

        // 2nd caps filter - after converter
        capsfilter1 = gst_element_factory_make("capsfilter", NULL);
        GstCaps* caps1 = gst_caps_new_simple("video/x-raw(memory:NVMM)",
                                             "format", G_TYPE_STRING, "NV12",
                                             NULL);
        g_object_set(G_OBJECT(capsfilter1), "caps", caps1, NULL);
        gst_caps_unref(caps1);

        // Queue before encoder
        queue0 = gst_element_factory_make("queue", NULL);
        g_object_set(G_OBJECT(queue0), "max-size-buffers", 3, "leaky", 2, NULL); // downstream leaky

        // Encoder
        std::string enc_name = "enc" + std::to_string(stream_id);
        encoder = gst_element_factory_make("nvv4l2h264enc", enc_name.c_str());
        g_object_set(encoder,
                 "control-rate", 1,
                 "bitrate", bitrate,
                 "iframeinterval", 30,
                 "num-B-Frames", 0,
                 "preset-level", 1,
                 "profile", 0,
                 "maxperf-enable", 1,
                 "insert-sps-pps", 1,
                 "insert-vui", 1,
                 "EnableTwopassCBR", 0,
                 NULL);

        // Queue after encoder
        queue1 = gst_element_factory_make("queue", NULL);
        g_object_set(G_OBJECT(queue1), "max-size-buffers", 5, "leaky", 2, NULL); // downstream leaky

        // Parser
        parse = gst_element_factory_make("h264parse", NULL);
        g_object_set(parse, "config-interval", 0, NULL);

        // Payloader
        payloader = gst_element_factory_make("rtph264pay", NULL);
        g_object_set(payloader,
                    "pt", 96,
                    "mtu", 1200,
                    "config-interval", -1,
                    NULL);

        // 3rd caps filter - after payloader
        capsfilter2 = gst_element_factory_make("capsfilter", NULL);
        GstCaps* caps2 = gst_caps_new_simple("application/x-rtp",
                                             "media", G_TYPE_STRING, "video",
                                             "encoding-name", G_TYPE_STRING, "H264",
                                             "payload", G_TYPE_INT, 96,
                                             NULL);
        g_object_set(G_OBJECT(capsfilter2), "caps", caps2, NULL);
        gst_caps_unref(caps2);

        // Add elements to pipeline
        gst_bin_add_many(GST_BIN(pipeline), src, capsfilter0, convert, capsfilter1, 
                         queue0, encoder, queue1, parse, payloader, capsfilter2, NULL);
        
        // Link elements
        gst_element_link_many(src, capsfilter0, convert, capsfilter1, 
                              queue0, encoder, queue1, parse, payloader, capsfilter2, NULL);

        // Link to webrtcbin
        GstPad* pay_src = gst_element_get_static_pad(capsfilter2, "src");
        GstPad* webrtc_sink = gst_element_get_request_pad(webrtcbin, "sink_%u"); // request new sink pad
        gst_pad_link(pay_src, webrtc_sink);
        gst_object_unref(pay_src);
        gst_object_unref(webrtc_sink);
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
        GstElement* webrtcbin = gst_element_factory_make("webrtcbin", "webrtcbin");
        if (!webrtcbin) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create webrtcbin element.");
            return;
        }

        // Set webrtcbin properties
        g_object_set(webrtcbin,
             "bundle-policy", 1,          // GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE
             "latency", 0,
             "stun-server", "stun://stun.l.google.com:19302",
             NULL);
        
        // Add webrtcbin to pipeline
        gst_bin_add(GST_BIN(pipeline), webrtcbin);

        // Add media streams to the pipeline
        add_stream(0, "/dev/cam-arducam", "v4l2src", 640, 480, 30, 2000000);

        // pipeline = gst_parse_launch("webrtcbin name=webrtcbin bundle-policy=max-bundle latency=0 \
        //     stun-server=stun://stun.l.google.com:19302 \
        //     v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 ! \
        //         nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! \
        //         queue max-size-buffers=3 leaky=downstream ! \
        //         nvv4l2h264enc name=enc0 \
        //             control-rate=1 \
        //             bitrate=2000000 \
        //             iframeinterval=30 \
        //             num-B-Frames=0 \
        //             preset-level=1 \
        //             profile=0 \
        //             maxperf-enable=1 \
        //             insert-sps-pps=1 \
        //             insert-vui=1 \
        //             EnableTwopassCBR=0 ! \
        //         queue max-size-buffers=5 leaky=downstream ! \
        //         h264parse config-interval=0 ! \
        //         rtph264pay name=pay0 \
        //             pt=96 \
        //             mtu=1200 \
        //             config-interval=-1 ! \
        //         application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin. \
        //     v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 ! \
        //         nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! \
        //         queue max-size-buffers=3 leaky=downstream ! \
        //         nvv4l2h264enc name=enc1 \
        //             control-rate=1 \
        //             bitrate=2000000 \
        //             iframeinterval=30 \
        //             num-B-Frames=0 \
        //             preset-level=1 \
        //             profile=0 \
        //             maxperf-enable=1 \
        //             insert-sps-pps=1 \
        //             insert-vui=1 \
        //             EnableTwopassCBR=0 ! \
        //         queue max-size-buffers=5 leaky=downstream ! \
        //         h264parse config-interval=0 ! \
        //         rtph264pay name=pay1 \
        //             pt=96 \
        //             mtu=1200 \
        //             config-interval=-1 ! \
        //         application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin.",
        //     &error);

        // webrtcbin name=webrtcbin bundle-policy=max-bundle latency=0 \
        //     stun-server=stun://stun.l.google.com:19302 \
        //         v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 \
        //             ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true insert-sps-pps=true \
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! h264parse ! rtph264pay config-interval=1 pt=96 \
        //             ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin. \
        //         v4l2src device=/dev/cam-aveo ! video/x-raw,width=640,height=480,framerate=30/1 \
        //             ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true insert-sps-pps=true \
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! h264parse ! rtph264pay config-interval=1 pt=96 \
        //             ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin. \
        //         nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 \
        //             ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 \
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true insert-sps-pps=true\
        //             ! queue max-size-buffers=2 leaky=downstream \
        //             ! h264parse ! rtph264pay config-interval=1 pt=96 \
        //             ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin.
        
        // pipeline = gst_parse_launch("webrtcbin name=webrtcbin bundle-policy=max-bundle latency=0 \
        //     stun-server=stun://stun.l.google.com:19302 \
        //     input-selector name=input_selector0 \
        //         ! queue max-size-buffers=2 leaky=downstream \
        //         ! nvv4l2h264enc bitrate=2500000 iframeinterval=30 control-rate=1 preset-level=1 profile=2 maxperf-enable=true \
        //         ! queue max-size-buffers=2 leaky=downstream \
        //         ! h264parse ! rtph264pay config-interval=1 pt=96 \
        //         ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtcbin. \
        //     v4l2src device=/dev/cam-arducam ! video/x-raw,width=640,height=480,framerate=30/1 ! nvvidconv name=nvvidconv0 ! video/x-raw(memory:NVMM),format=NV12 ! input_selector0.sink_0 \
        //     videotestsrc is-live=true ! video/x-raw,width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! input_selector0.sink_1",
        //     &error);

        if (error) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create GStreamer pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        // if (!pipeline) {
        //     RCLCPP_ERROR(this->get_logger(), "ERROR: Could not create GStreamer pipeline.");
        //     return;
        // }

        // // Get webrtcbin element
        // webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "webrtcbin");
        // if (!webrtcbin) {
        //     RCLCPP_ERROR(this->get_logger(), "ERROR: Could not get WebRTC element.");
        //     return;
        // }

        enc0 = gst_bin_get_by_name(GST_BIN(pipeline), "enc0");
        enc1 = gst_bin_get_by_name(GST_BIN(pipeline), "enc1");
        if (!enc0 || !enc1) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not get one or more encoder elements.");
            return;
        } else {
            // Read initial bitrate
            gint64 val = 0;
            g_object_get(G_OBJECT(enc0), "bitrate", &val, nullptr);
            current_bitrate = static_cast<int>(val);

            // Start bitrate controller
            bitrate_running = true;
            bitrate_thread = std::thread(&WebRTCSendNode::bitrate_controller_loop, this);
        }

        // Get payloader elements
        GstElement* pay0 = gst_bin_get_by_name(GST_BIN(pipeline), "pay0");
        GstElement* pay1 = gst_bin_get_by_name(GST_BIN(pipeline), "pay1");
        if (!pay0 || !pay1) {
            RCLCPP_ERROR(get_logger(), "Failed to get payloader elements");
            return;
        }

        // Get payloader src pads
        GstPad* pay0_src = gst_element_get_static_pad(pay0, "src");
        GstPad* pay1_src = gst_element_get_static_pad(pay1, "src");
        if (!pay0_src || !pay1_src) {
            RCLCPP_ERROR(get_logger(), "Failed to get payloader src pads");
            return;
        }

        // Request sink pads from webrtcbin for each stream
        GstPad* sink0 = gst_element_get_request_pad(webrtcbin, "sink_%u");
        GstPad* sink1 = gst_element_get_request_pad(webrtcbin, "sink_%u");
        if (!sink0 || !sink1) {
            RCLCPP_ERROR(get_logger(), "Failed to request sink pads from webrtcbin");
            return;
        }

        // Link payloader src pads to webrtcbin sink pads
        if (GST_PAD_LINK_FAILED(gst_pad_link(pay0_src, sink0))) {
            RCLCPP_ERROR(get_logger(), "Failed to link pay0 to webrtcbin");
        }
        if (GST_PAD_LINK_FAILED(gst_pad_link(pay1_src, sink1))) {
            RCLCPP_ERROR(get_logger(), "Failed to link pay1 to webrtcbin");
        }

        // Cleanup references
        gst_object_unref(pay0_src);
        gst_object_unref(pay1_src);
        gst_object_unref(sink0);
        gst_object_unref(sink1);
        gst_object_unref(pay0);
        gst_object_unref(pay1);

        gst_pipeline_use_clock(GST_PIPELINE(pipeline), gst_system_clock_obtain());

        // Set WebRTC properties
        g_object_set(G_OBJECT(webrtcbin), "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(&WebRTCSendNode::on_negotiation_needed), this);
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(&WebRTCSendNode::send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(&WebRTCSendNode::on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void handle_receiver_stats(const Json::Value& stats) {
        // Get individual metrics and update member variables
        double l_jitter = stats.isMember("jitter") ? stats["jitter"].asDouble() : -1.0;
        int64_t l_packets_received = stats.isMember("packets_received") ? stats["packets_received"].asUInt64() : -1;
        int64_t l_packets_lost = stats.isMember("packets_lost") ? stats["packets_lost"].asUInt64() : -1;
        uint64_t l_bytes_received = stats.isMember("bytes_received") ? stats["bytes_received"].asUInt64() : -1;

        // Print or log stats
        RCLCPP_INFO(this->get_logger(),
            "Received Stats -> Jitter: %.4f | Bytes: %lu | Packets: %lu/%lu",
            l_jitter, l_bytes_received, l_packets_received, l_packets_lost
        );
    }

    void bitrate_controller_loop() {
        using namespace std::chrono;
        const milliseconds interval(2000); // 2 seconds
        int consecutive_lost = 0;
    }

    void change_bitrate(int new_bitrate) {
        // Capture 'this' to access the private member 'enc'
        auto func = [this, new_bitrate]() {
            if (this->enc0) {
                g_object_set(G_OBJECT(this->enc0),
                            "bitrate",
                            static_cast<gint64>(new_bitrate),
                            nullptr);
                RCLCPP_INFO(this->get_logger(), "Encoder bitrate set to %d bps", new_bitrate);
            } else {
                RCLCPP_WARN(this->get_logger(), "Encoder element missing, cannot set bitrate");
            }
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

    // Bitrate controller
    std::thread bitrate_thread;
    int current_bitrate{2000000};
    const int min_bitrate{800000};
    const int max_bitrate{2000000};
    const int bitrate_step{100000};

    std::atomic<double> jitter{-1.0};
    std::atomic<int64_t> packets_received{-1};
    std::atomic<int64_t> packets_lost{-1};
    std::atomic<int64_t> bytes_received{-1};
    std::atomic<uint64_t> last_bytes_received{0};

    GstElement* enc0;                     // Encoder for first camera
    GstElement* enc1;                     // Encoder for second camera
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