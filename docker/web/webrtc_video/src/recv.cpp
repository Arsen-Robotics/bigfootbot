#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <jsoncpp/json/json.h>  // for WebSocket message handling (JSON)

class WebRTCRecv {
public:
    WebRTCRecv() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        this->pipeline = nullptr;

        // Setup GStreamer pipeline
        setup_pipeline();
    }

    ~WebRTCRecv() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
    }

    void connect() {
        // Set up WebSocket connection using websocketpp
        websocketpp::client<websocketpp::config::asio_client> client;
        client.init_asio();

        client.set_open_handler([this](websocketpp::connection_hdl hdl) {
            websocket_handle = hdl;
            std::cout << "Connected to signaling server" << std::endl;
            sendHello();
        });

        client.set_message_handler([this](websocketpp::connection_hdl hdl, websocketpp::config::asio_client::message_type::ptr msg) {
            handleMessage(msg->get_payload());
        });

        try {
            websocket = &client;
            client.set_access_channels(websocketpp::log::alevel::none);
            client.set_error_channels(websocketpp::log::alevel::none);
            client.set_message_handler([this](websocketpp::connection_hdl hdl, websocketpp::config::asio_client::message_type::ptr msg) {
                handleMessage(msg->get_payload());
            });
            client.connect("ws://87.119.173.184:8765");
            client.run();
        } catch (const std::exception& e) {
            std::cout << "Failed to connect: " << e.what() << std::endl;
        }
    }

    void sendHello() {
        Json::Value message;
        message["message"] = "HELLO";
        sendMessage(message);
    }

    void start_pipeline() {
        // Create GStreamer pipeline
        pipeline = gst_parse_launch("webrtcbin name=recvonly bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 audiotestsrc ! audioconvert ! fakesink", nullptr);

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
        g_object_set(G_OBJECT(webrtcbin), "latency", 0, "bundle-policy", "max-bundle", "stun-server", "stun://stun.l.google.com:19302", nullptr);

        // Connect to signals
        g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(send_ice_candidate), this);
        g_signal_connect(webrtcbin, "pad-added", G_CALLBACK(on_incoming_stream), this);

        // Set pipeline state to PLAYING
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void send_ice_candidate(GstElement* webrtcbin, guint mlineindex, gchar* candidate, gpointer user_data) {
        std::cout << "Sending ICE candidate: " << candidate << std::endl;

        // Create a JSON message to send over WebSocket
        Json::Value icemsg;
        icemsg["ice"]["candidate"] = candidate;
        icemsg["ice"]["sdpMLineIndex"] = static_cast<int>(mlineindex);  // Casting mlineindex to int for JSON

        // Convert JSON message to string
        Json::StreamWriterBuilder writer;
        std::string icemsg_str = Json::writeString(writer, icemsg);

        // Send the ICE candidate over WebSocket (this part would need to use your WebSocket client)
        // Example of sending over WebSocket (using an async WebSocket library like websocketpp)
        self->send_ws(icemsg_str);
    }

    void send_ws(const std::string& msg) {
        std::lock_guard<std::mutex> lock(ws_mutex); // Ensure thread safety when accessing WebSocket

        if (ws_client) {
            websocketpp::lib::error_code ec;
            ws_client->send(hdl, msg, websocketpp::frame::opcode::text, ec);
            if (ec) {
                std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
            } else {
                std::cout << "Sent ICE message over WebSocket: " << msg << std::endl;
            }
        } else {
            std::cerr << "WebSocket client is not initialized!" << std::endl;
        }
    }