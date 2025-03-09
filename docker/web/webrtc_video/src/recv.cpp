#include <iostream>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <jsoncpp/json/json.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

class WebRTCRecv {
public:
    WebRTCRecv() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        this->pipeline = nullptr;
        this->webrtcbin = nullptr;
    }

    ~WebRTCRecv() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
        if (webrtcbin) {
            gst_object_unref(webrtcbin);
        }
    }

    void connect() {
        try {
            // Set up WebSocket connection using websocketpp
            websocketpp::client<websocketpp::config::asio_client> client;
            client.init_asio();

            client.set_open_handler(std::bind(&WebRTCRecv::on_open, this, std::placeholders::_1, &client));
            client.set_message_handler(std::bind(&WebRTCRecv::on_msg, this, std::placeholders::_1, std::placeholders::_2));

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

    void on_open(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>* c) {
        std::cout << "Connected! Sending HELLO message..." << std::endl;

        global_hdl = hdl;
        global_client = c;

        // Create a JSON object to send "HELLO" message
        Json::Value msg;
        msg["status"] = "HELLO";

        // Convert to string
        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, msg);

        // Send the initial message
        send_ws(message);
    }

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
                std::cout << "Received OK message from SEND peer." << std::endl;
                setup_pipeline();
            } else if (jsonMsg.isMember("ice")) {
                handle_ice(payload);
            } else if (jsonMsg.isMember("sdp")) {
                handle_sdp(payload);
            } else {
                std::cout << "Unknown JSON message type" << std::endl;
            }
        } else {
            std::cout << "Failed to parse JSON: " << errs << std::endl;
        }
    }

    void send_ws(const std::string& msg) {
        if (global_client) {
            websocketpp::lib::error_code ec;
            global_client->send(global_hdl, msg, websocketpp::frame::opcode::text, ec);
            if (ec) {
                std::cerr << "Error sending WebSocket message: " << ec.message() << std::endl;
            } else {
                std::cout << "Sent message over WebSocket: " << msg << std::endl;
            }
        } else {
            std::cerr << "WebSocket client is not initialized!" << std::endl;
        }
    }

    void setup_pipeline() {
        // Create GStreamer pipeline
        pipeline = gst_parse_launch("webrtcbin name=recvonly stun-server=stun://stun.l.google.com:19302 audiotestsrc ! audioconvert ! fakesink", nullptr);

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
        send_ws(icemsg_str);
    }

    void handle_sdp(const std::string& sdp) {
        // Handle SDP message
    }

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

private:
    GstElement* pipeline;
    GstElement* webrtcbin;
    websocketpp::connection_hdl global_hdl;
    websocketpp::client<websocketpp::config::asio_client>* global_client;
};

int main() {
    WebRTCRecv recv;
    recv.connect();

    return 0;
}