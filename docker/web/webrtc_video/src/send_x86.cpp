/**
 * WebRTC Video Sender for x86 platforms
 * 
 * This application captures video from a camera device and streams it
 * using WebRTC to a browser client through a signaling server.
 */

#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/sdp.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <csignal>
#include <getopt.h>

/**
 * WebRTCSender class handles video capture and WebRTC streaming
 */
class WebRTCSender {
private:
    GstElement* m_pipeline;        // Main GStreamer pipeline
    GstElement* m_webrtcbin;       // WebRTC element within pipeline
    GMainLoop* m_loop;             // GStreamer main loop
    std::atomic<bool> m_running;   // Flag indicating if sender is running
    
    // WebSocket variables for signaling
    websocketpp::client<websocketpp::config::asio_client>* m_client;
    websocketpp::connection_hdl m_hdl;
    std::string m_signalingServer;
    
    // Video source configuration
    std::string m_device;          // Camera device path
    int m_width;                   // Video width
    int m_height;                  // Video height
    int m_framerate;               // Video framerate
    
    // Thread management for async operations
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::queue<std::string> m_msgQueue;
    std::thread m_wsThread;        // WebSocket client thread
    std::atomic<bool> m_wsRunning; // Flag for WebSocket thread status

public:
    /**
     * Constructor with default configuration values
     */
    WebRTCSender() : 
        m_pipeline(nullptr), 
        m_webrtcbin(nullptr),
        m_loop(nullptr), 
        m_running(false),
        m_client(nullptr),
        m_signalingServer("ws://127.0.0.1:8765"),
        m_device("/dev/video0"),
        m_width(640),
        m_height(480),
        m_framerate(30),
        m_wsRunning(false) {
    }

    /**
     * Destructor ensures proper cleanup
     */
    ~WebRTCSender() {
        stop();
    }

    /**
     * Parse command line arguments to configure the sender
     * 
     * @param argc Argument count
     * @param argv Argument values
     */
    void parseArgs(int argc, char** argv) {
        int opt;
        while ((opt = getopt(argc, argv, "d:w:h:f:s:")) != -1) {
            switch (opt) {
                case 'd':
                    m_device = optarg;
                    break;
                case 'w':
                    m_width = std::stoi(optarg);
                    break;
                case 'h':
                    m_height = std::stoi(optarg);
                    break;
                case 'f':
                    m_framerate = std::stoi(optarg);
                    break;
                case 's':
                    m_signalingServer = optarg;
                    break;
                default:
                    std::cerr << "Unknown option: " << static_cast<char>(opt) << std::endl;
                    break;
            }
        }
    }
    
    /**
     * Connect to the signaling server via WebSocket
     */
    void connect() {
        try {
            std::cout << "Connecting to signaling server: " << m_signalingServer << std::endl;
            
            // Initialize WebSocket client
            m_client = new websocketpp::client<websocketpp::config::asio_client>();
            m_client->clear_access_channels(websocketpp::log::alevel::all);
            m_client->set_access_channels(websocketpp::log::alevel::connect);
            m_client->set_access_channels(websocketpp::log::alevel::disconnect);
            m_client->set_access_channels(websocketpp::log::alevel::app);
            
            m_client->init_asio();
            
            // Set up event handlers
            m_client->set_open_handler([this](websocketpp::connection_hdl hdl) {
                onOpen(hdl);
            });
            
            m_client->set_message_handler([this](websocketpp::connection_hdl hdl, 
                                          websocketpp::client<websocketpp::config::asio_client>::message_ptr msg) {
                onMessage(hdl, msg);
            });
            
            // Create connection
            websocketpp::lib::error_code ec;
            auto con = m_client->get_connection(m_signalingServer, ec);
            
            if (ec) {
                std::cerr << "Failed to create connection: " << ec.message() << std::endl;
                return;
            }
            
            m_client->connect(con);
            
            // Start WebSocket thread
            m_wsRunning = true;
            m_wsThread = std::thread([this]() {
                try {
                    m_client->run();
                } catch (const std::exception& e) {
                    std::cerr << "WebSocket thread exception: " << e.what() << std::endl;
                }
            });
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to connect to signaling server: " << e.what() << std::endl;
        }
    }
    
    /**
     * Handler for successful WebSocket connection
     * 
     * @param hdl WebSocket connection handle
     */
    void onOpen(websocketpp::connection_hdl hdl) {
        std::cout << "Connected to signaling server" << std::endl;
        m_hdl = hdl;
        
        // Send HELLO message to register with the server
        Json::Value msg;
        msg["status"] = "HELLO";
        
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);
        
        sendMessage(message);
    }
    
    /**
     * Handler for incoming WebSocket messages
     * 
     * @param hdl WebSocket connection handle
     * @param msg Received message
     */
    void onMessage(websocketpp::connection_hdl hdl, 
                  websocketpp::client<websocketpp::config::asio_client>::message_ptr msg) {
        std::string payload = msg->get_payload();
        std::cout << "Received message: " << payload << std::endl;
        
        // Parse JSON message
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(payload);
        
        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            if (jsonMsg.isMember("status")) {
                std::string status = jsonMsg["status"].asString();
                if ((status == "OK" || status == "HELLO") && !m_pipeline) {
                    // Only start the pipeline if it's not already started
                    setupPipeline();
                }
            } else if (jsonMsg.isMember("ice")) {
                // Received ICE candidate
                handleIceCandidate(jsonMsg);
            } else if (jsonMsg.isMember("sdp") && jsonMsg["type"].asString() == "answer") {
                // Received SDP answer
                handleSdpAnswer(jsonMsg);
            } else {
                std::cout << "Unknown message type" << std::endl;
            }
        } else {
            std::cerr << "Failed to parse JSON: " << errs << std::endl;
        }
    }
    
    /**
     * Send a message to the signaling server
     * 
     * @param message JSON message to send
     */
    void sendMessage(const std::string& message) {
        if (!m_client) return;
        
        try {
            websocketpp::lib::error_code ec;
            m_client->send(m_hdl, message, websocketpp::frame::opcode::text, ec);
            
            if (ec) {
                std::cerr << "Error sending message: " << ec.message() << std::endl;
            } else {
                std::cout << "Sent message: " << message << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception while sending message: " << e.what() << std::endl;
        }
    }
    
    /**
     * Callback for ICE candidate generation
     * 
     * @param webrtc WebRTC element
     * @param mlineIndex Media line index
     * @param candidate ICE candidate string
     * @param userData Pointer to WebRTCSender instance
     */
    static void onIceCandidate(GstElement* webrtc, guint mlineIndex, gchar* candidate, gpointer userData) {
        WebRTCSender* self = static_cast<WebRTCSender*>(userData);
        
        // Create JSON message with ICE candidate
        Json::Value ice;
        ice["ice"]["candidate"] = candidate;
        ice["ice"]["sdpMLineIndex"] = mlineIndex;
        
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, ice);
        
        self->sendMessage(message);
    }
    
    /**
     * Callback for WebRTC negotiation needed event
     * 
     * @param element WebRTC element
     * @param userData Pointer to WebRTCSender instance
     */
    static void onNegotiationNeeded(GstElement* element, gpointer userData) {
        WebRTCSender* self = static_cast<WebRTCSender*>(userData);
        
        // Create SDP offer
        GstPromise* promise = gst_promise_new_with_change_func(onOfferCreated, userData, nullptr);
        g_signal_emit_by_name(self->m_webrtcbin, "create-offer", nullptr, promise);
    }
    
    /**
     * Callback for SDP offer creation
     * 
     * @param promise GStreamer promise
     * @param userData Pointer to WebRTCSender instance
     */
    static void onOfferCreated(GstPromise* promise, gpointer userData) {
        WebRTCSender* self = static_cast<WebRTCSender*>(userData);
        
        // Get the offer from the promise
        GstWebRTCSessionDescription* offer = nullptr;
        const GstStructure* reply = gst_promise_get_reply(promise);
        gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
        gst_promise_unref(promise);
        
        // Set local description
        GstPromise* setLocalPromise = gst_promise_new();
        g_signal_emit_by_name(self->m_webrtcbin, "set-local-description", offer, setLocalPromise);
        gst_promise_interrupt(setLocalPromise);
        gst_promise_unref(setLocalPromise);
        
        // Create and send SDP offer
        char* sdpStr = gst_sdp_message_as_text(offer->sdp);
        
        Json::Value msg;
        msg["sdp"] = sdpStr;
        msg["type"] = "offer";
        
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);
        
        self->sendMessage(message);
        
        // Cleanup
        g_free(sdpStr);
        gst_webrtc_session_description_free(offer);
    }
    
    /**
     * Handle incoming ICE candidate from remote peer
     * 
     * @param jsonMsg JSON message containing ICE candidate
     */
    void handleIceCandidate(const Json::Value& jsonMsg) {
        if (!m_webrtcbin) return;
        
        const Json::Value& ice = jsonMsg["ice"];
        const std::string& candidate = ice["candidate"].asString();
        int sdpMLineIndex = ice["sdpMLineIndex"].asInt();
        
        // Add ICE candidate to WebRTC element
        g_signal_emit_by_name(m_webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
    }
    
    /**
     * Handle incoming SDP answer from remote peer
     * 
     * @param message JSON message containing SDP answer
     */
    void handleSdpAnswer(const Json::Value& message) {
        if (!message.isMember("sdp") || !message.isMember("type")) {
            std::cerr << "Invalid SDP message format" << std::endl;
            return;
        }

        if (message["type"].asString() != "answer") {
            std::cerr << "Unexpected SDP type: " << message["type"].asString() << std::endl;
            return;
        }

        std::string sdp = message["sdp"].asString();
        
        GstSDPMessage *sdp_msg;
        gst_sdp_message_new(&sdp_msg);
        gst_sdp_message_parse_buffer((guint8 *) sdp.c_str(), sdp.length(), sdp_msg);

        GstWebRTCSessionDescription *answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
        
        g_signal_emit_by_name(m_webrtcbin, "set-remote-description", answer, NULL);
        gst_webrtc_session_description_free(answer);
    }
    
    /**
     * Set up the GStreamer pipeline for video capture and WebRTC streaming
     */
    void setupPipeline() {
        // Create GStreamer main loop
        m_loop = g_main_loop_new(nullptr, FALSE);
        
        // Build pipeline description string
        std::string pipelineDesc = 
            "webrtcbin name=webrtc bundle-policy=max-bundle "
            "stun-server=stun://stun.l.google.com:19302 "
            "v4l2src device=" + m_device + " ! "
            "video/x-raw,width=" + std::to_string(m_width) + ",height=" + std::to_string(m_height) + ",framerate=" + std::to_string(m_framerate) + "/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency key-int-max=30 bitrate=1000 speed-preset=ultrafast ! "
            "h264parse config-interval=-1 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "webrtc.";
        
        std::cout << "Creating pipeline: " << pipelineDesc << std::endl;
        
        GError* error = nullptr;
        m_pipeline = gst_parse_launch(pipelineDesc.c_str(), &error);
        
        if (error) {
            std::cerr << "Failed to create pipeline: " << error->message << std::endl;
            g_error_free(error);
            return;
        }
        
        // Get webrtcbin element
        m_webrtcbin = gst_bin_get_by_name(GST_BIN(m_pipeline), "webrtc");
        
        // Connect WebRTC signals
        g_signal_connect(m_webrtcbin, "on-ice-candidate", G_CALLBACK(onIceCandidate), this);
        g_signal_connect(m_webrtcbin, "on-negotiation-needed", G_CALLBACK(onNegotiationNeeded), this);
        
        // Start playing
        gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
        
        m_running = true;
        std::cout << "WebRTC pipeline started" << std::endl;
        
        // Start pipeline loop in a separate thread
        std::thread loop_thread([this]() {
            g_main_loop_run(m_loop);
        });
        loop_thread.detach();
    }

    bool start() {
        if (m_running) {
            std::cout << "WebRTC sender already running" << std::endl;
            return false;
        }
        
        connect();
        return true;
    }

    void stop() {
        m_running = false;
        
        // Stop WebSocket thread
        if (m_wsRunning) {
            m_wsRunning = false;
            if (m_client) {
                m_client->stop();
            }
            if (m_wsThread.joinable()) {
                m_wsThread.join();
            }
            delete m_client;
            m_client = nullptr;
        }
        
        // Stop GStreamer pipeline
        if (m_pipeline) {
            gst_element_set_state(m_pipeline, GST_STATE_NULL);
            gst_object_unref(m_pipeline);
            m_pipeline = nullptr;
        }
        
        if (m_webrtcbin) {
            gst_object_unref(m_webrtcbin);
            m_webrtcbin = nullptr;
        }
        
        if (m_loop) {
            g_main_loop_quit(m_loop);
            g_main_loop_unref(m_loop);
            m_loop = nullptr;
        }
        
        std::cout << "WebRTC sender stopped" << std::endl;
    }

    bool isRunning() const {
        return m_running;
    }
};

// Global signal handler variables
std::atomic<bool> g_shutdownRequested(false);
WebRTCSender* g_sender = nullptr;

void signalHandler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    g_shutdownRequested = true;
    if (g_sender) {
        g_sender->stop();
    }
}

void setupSignalHandlers() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -d <device>      Video device (default: /dev/video0)" << std::endl;
    std::cout << "  -w <width>       Video width (default: 640)" << std::endl;
    std::cout << "  -h <height>      Video height (default: 480)" << std::endl;
    std::cout << "  -f <framerate>   Video framerate (default: 30)" << std::endl;
    std::cout << "  -s <server>      Signaling server URL (default: ws://127.0.0.1:8765)" << std::endl;
}

int main(int argc, char** argv) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Setup signal handlers
    setupSignalHandlers();

    // Create WebRTC sender
    WebRTCSender sender;
    g_sender = &sender;

    // Parse command line arguments
    sender.parseArgs(argc, argv);

    // Start the sender
    if (!sender.start()) {
        std::cerr << "Failed to start WebRTC sender" << std::endl;
        return 1;
    }

    // Print usage information
    printUsage(argv[0]);

    // Wait for signal to stop
    while (!g_shutdownRequested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    gst_deinit();
    return 0;
} 