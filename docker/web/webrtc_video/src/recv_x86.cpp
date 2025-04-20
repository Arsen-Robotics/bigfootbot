#include <iostream>
#include <string>
#include <csignal>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/sdp.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <queue>
#include <getopt.h>

class WebRTCReceiver {
private:
    GstElement* m_pipeline;
    GstElement* m_webrtcbin;
    GMainLoop* m_loop;
    std::atomic<bool> m_running;
    
    // WebSocket variables
    websocketpp::client<websocketpp::config::asio_client>* m_client;
    websocketpp::connection_hdl m_hdl;
    std::string m_signalingServer;
    
    // Display settings
    std::string m_displaySink;
    
    // Thread management
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::queue<std::string> m_msgQueue;
    std::thread m_wsThread;
    std::atomic<bool> m_wsRunning;

public:
    WebRTCReceiver() : 
        m_pipeline(nullptr),
        m_webrtcbin(nullptr),
        m_loop(nullptr),
        m_running(false),
        m_client(nullptr),
        m_signalingServer("ws://127.0.0.1:8765"),
        m_displaySink("autovideosink"),
        m_wsRunning(false) {}

    ~WebRTCReceiver() {
        stop();
    }
    
    void parseArgs(int argc, char** argv) {
        int opt;
        while ((opt = getopt(argc, argv, "s:d:")) != -1) {
            switch (opt) {
                case 's':
                    m_signalingServer = optarg;
                    break;
                case 'd':
                    m_displaySink = optarg;
                    break;
                default:
                    std::cerr << "Unknown option: " << static_cast<char>(opt) << std::endl;
                    break;
            }
        }
    }
    
    void connect() {
        try {
            std::cout << "Connecting to signaling server: " << m_signalingServer << std::endl;
            
            m_client = new websocketpp::client<websocketpp::config::asio_client>();
            m_client->clear_access_channels(websocketpp::log::alevel::all);
            m_client->set_access_channels(websocketpp::log::alevel::connect);
            m_client->set_access_channels(websocketpp::log::alevel::disconnect);
            m_client->set_access_channels(websocketpp::log::alevel::app);
            
            m_client->init_asio();
            
            // Set up handlers
            m_client->set_open_handler([this](websocketpp::connection_hdl hdl) {
                onOpen(hdl);
            });
            
            m_client->set_message_handler([this](websocketpp::connection_hdl hdl, 
                                          websocketpp::client<websocketpp::config::asio_client>::message_ptr msg) {
                onMessage(hdl, msg);
            });
            
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
    
    void onOpen(websocketpp::connection_hdl hdl) {
        std::cout << "Connected to signaling server" << std::endl;
        m_hdl = hdl;
        
        // Set up the pipeline
        setupPipeline();
        
        // Send HELLO message
        Json::Value msg;
        msg["status"] = "HELLO";
        
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);
        
        sendMessage(message);
    }
    
    void onMessage(websocketpp::connection_hdl hdl, 
                  websocketpp::client<websocketpp::config::asio_client>::message_ptr msg) {
        std::string payload = msg->get_payload();
        std::cout << "Received message: " << payload << std::endl;
        
        Json::CharReaderBuilder reader;
        Json::Value jsonMsg;
        std::string errs;
        std::istringstream s(payload);
        
        if (Json::parseFromStream(reader, s, &jsonMsg, &errs)) {
            if (jsonMsg.isMember("status") && jsonMsg["status"].asString() == "OK") {
                // Server acknowledged our HELLO
                std::cout << "Signaling server acknowledged connection" << std::endl;
            } else if (jsonMsg.isMember("ice")) {
                // Received ICE candidate
                handleIceCandidate(jsonMsg);
            } else if (jsonMsg.isMember("sdp") && jsonMsg["type"].asString() == "offer") {
                // Received SDP offer
                handleSdpOffer(jsonMsg);
            } else {
                std::cout << "Unknown message type" << std::endl;
            }
        } else {
            std::cerr << "Failed to parse JSON: " << errs << std::endl;
        }
    }
    
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
    
    static void onIceCandidate(GstElement* webrtc, guint mlineIndex, gchar* candidate, gpointer userData) {
        WebRTCReceiver* self = static_cast<WebRTCReceiver*>(userData);
        
        Json::Value ice;
        ice["ice"]["candidate"] = candidate;
        ice["ice"]["sdpMLineIndex"] = mlineIndex;
        
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, ice);
        
        self->sendMessage(message);
    }
    
    static void onPadAdded(GstElement* element, GstPad* pad, gpointer userData) {
        WebRTCReceiver* self = static_cast<WebRTCReceiver*>(userData);
        
        // Check the name of the new pad
        gchar* name = gst_pad_get_name(pad);
        std::cout << "New pad: " << name << std::endl;
        
        // If it's a video pad, link it to the decoder
        if (g_str_has_prefix(name, "recv_rtp_src_0")) {
            // Get decodebin element and link the pad to it
            GstElement* decodebin = gst_bin_get_by_name(GST_BIN(self->m_pipeline), "decode");
            if (decodebin) {
                GstPad* sinkpad = gst_element_get_static_pad(decodebin, "sink");
                GstPadLinkReturn ret = gst_pad_link(pad, sinkpad);
                if (GST_PAD_LINK_FAILED(ret)) {
                    std::cerr << "Failed to link pads" << std::endl;
                } else {
                    std::cout << "Linked video pad to decoder" << std::endl;
                }
                gst_object_unref(sinkpad);
            }
        }
        
        g_free(name);
    }
    
    static void onDecodebinPadAdded(GstElement* element, GstPad* pad, gpointer userData) {
        WebRTCReceiver* self = static_cast<WebRTCReceiver*>(userData);
        
        // Check the capabilities of the pad
        GstCaps* caps = gst_pad_get_current_caps(pad);
        const GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);
        
        if (g_str_has_prefix(name, "video")) {
            // The pad has video data, link to videoconvert
            GstElement* convert = gst_bin_get_by_name(GST_BIN(self->m_pipeline), "vidconv");
            if (convert) {
                GstPad* sinkpad = gst_element_get_static_pad(convert, "sink");
                GstPadLinkReturn ret = gst_pad_link(pad, sinkpad);
                if (GST_PAD_LINK_FAILED(ret)) {
                    std::cerr << "Failed to link video pad to converter" << std::endl;
                } else {
                    std::cout << "Linked video pad to converter" << std::endl;
                }
                gst_object_unref(sinkpad);
            }
        }
        
        gst_caps_unref(caps);
    }
    
    void handleIceCandidate(const Json::Value& jsonMsg) {
        if (!m_webrtcbin) return;
        
        const Json::Value& ice = jsonMsg["ice"];
        const std::string& candidate = ice["candidate"].asString();
        int sdpMLineIndex = ice["sdpMLineIndex"].asInt();
        
        g_signal_emit_by_name(m_webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
    }
    
    void handleSdpOffer(const Json::Value& jsonMsg) {
        if (!m_webrtcbin) return;
        
        const std::string& sdpText = jsonMsg["sdp"].asString();
        
        // Create GstSDPMessage from SDP text
        GstSDPMessage* sdp;
        gst_sdp_message_new(&sdp);
        int ret = gst_sdp_message_parse_buffer((guint8*)sdpText.c_str(), sdpText.size(), sdp);
        
        if (ret != GST_SDP_OK) {
            std::cerr << "Failed to parse SDP message" << std::endl;
            return;
        }
        
        // Create GstWebRTCSessionDescription from SDP message
        GstWebRTCSessionDescription* offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp);
        
        // Set remote description
        GstPromise* promise = gst_promise_new_with_change_func([](GstPromise* promise, gpointer user_data) {
            WebRTCReceiver* self = static_cast<WebRTCReceiver*>(user_data);
            
            // Create answer
            GstPromise* answer_promise = gst_promise_new_with_change_func([](GstPromise* promise, gpointer user_data) {
                WebRTCReceiver* self = static_cast<WebRTCReceiver*>(user_data);
                
                // Get the answer description
                GstWebRTCSessionDescription* answer = nullptr;
                const GstStructure* reply = gst_promise_get_reply(promise);
                gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
                
                // Set local description
                GstPromise* local_promise = gst_promise_new();
                g_signal_emit_by_name(self->m_webrtcbin, "set-local-description", answer, local_promise);
                gst_promise_interrupt(local_promise);
                gst_promise_unref(local_promise);
                
                // Send SDP answer
                char* sdp_string = gst_sdp_message_as_text(answer->sdp);
                
                Json::Value msg;
                msg["sdp"] = sdp_string;
                msg["type"] = "answer";
                
                Json::StreamWriterBuilder writer;
                std::string message = Json::writeString(writer, msg);
                
                self->sendMessage(message);
                
                g_free(sdp_string);
                gst_webrtc_session_description_free(answer);
            }, self, nullptr);
            
            g_signal_emit_by_name(self->m_webrtcbin, "create-answer", nullptr, answer_promise);
        }, this, nullptr);
        
        g_signal_emit_by_name(m_webrtcbin, "set-remote-description", offer, promise);
        gst_webrtc_session_description_free(offer);
    }
    
    void setupPipeline() {
        // Create GStreamer pipeline
        m_loop = g_main_loop_new(nullptr, FALSE);
        
        std::string pipelineDesc = 
            "webrtcbin name=webrtc bundle-policy=max-bundle "
            "stun-server=stun://stun.l.google.com:19302 "
            "decodebin name=decode ! videoconvert name=vidconv ! " + m_displaySink;
        
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
        g_signal_connect(m_webrtcbin, "pad-added", G_CALLBACK(onPadAdded), this);
        
        // Connect decodebin signals
        GstElement* decodebin = gst_bin_get_by_name(GST_BIN(m_pipeline), "decode");
        g_signal_connect(decodebin, "pad-added", G_CALLBACK(onDecodebinPadAdded), this);
        gst_object_unref(decodebin);
        
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
            std::cout << "WebRTC receiver already running" << std::endl;
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
        
        std::cout << "WebRTC receiver stopped" << std::endl;
    }

    bool isRunning() const {
        return m_running;
    }
};

// Global signal handler variables
std::atomic<bool> g_shutdownRequested(false);
WebRTCReceiver* g_receiver = nullptr;

void signalHandler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    g_shutdownRequested = true;
    if (g_receiver) {
        g_receiver->stop();
    }
}

void setupSignalHandlers() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -s <server>     Signaling server URL (default: ws://127.0.0.1:8765)" << std::endl;
    std::cout << "  -d <display>    Display sink to use (default: autovideosink, alternatives: ximagesink, xvimagesink)" << std::endl;
}

int main(int argc, char** argv) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Setup signal handlers
    setupSignalHandlers();

    // Create WebRTC receiver
    WebRTCReceiver receiver;
    g_receiver = &receiver;

    // Parse command line arguments
    receiver.parseArgs(argc, argv);

    // Start the receiver
    if (!receiver.start()) {
        std::cerr << "Failed to start WebRTC receiver" << std::endl;
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