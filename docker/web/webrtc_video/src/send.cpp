#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>

using namespace boost::asio;
using json = nlohmann::json;

class WebRTCSend {
private:
    io_context io;
    ip::tcp::socket socket;
    std::shared_ptr<gst::Element> webrtcbin;
    std::shared_ptr<gst::Pipeline> pipeline;

public:
    WebRTCSend(io_context& io) : socket(io) {
        gst_init(nullptr, nullptr);
    }

    void connect(const std::string& signaling_server) {
        try {
            ip::tcp::resolver resolver(io);
            auto endpoints = resolver.resolve(signaling_server, "8765");
            connect(socket, endpoints);
            std::cout << "Connected to signaling server" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to connect to signaling server: " << e.what() << std::endl;
        }
    }

    void send_sdp_offer(const GstWebRTCSessionDescription* offer) {
        json msg;
        msg["sdp"]["type"] = "offer";
        msg["sdp"]["sdp"] = offer->sdp.as_text();

        try {
            std::string data = msg.dump();
            boost::asio::write(socket, boost::asio::buffer(data));
            std::cout << "Sent SDP Offer: " << data << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to send SDP offer: " << e.what() << std::endl;
        }
    }

    void on_offer_created(const GstPromise* promise) {
        // Handle offer creation
        GstWebRTCSessionDescription* offer = gst_webrtc_session_description_new(
            GST_WEBRTC_SDP_TYPE_OFFER, nullptr);
        
        // Set local description
        GstPromise* local_promise = gst_promise_new();
        gst_webrtcbin_emit(webrtcbin.get(), "set-local-description", offer, local_promise);
        gst_promise_wait(local_promise);
        send_sdp_offer(offer);
    }

    void start_pipeline() {
        // Set up the WebRTC pipeline
        pipeline = gst_pipeline_new("webrtc-pipeline");

        // Create the webrtcbin element and configure it
        webrtcbin = gst_element_factory_make("webrtcbin", "sendrecv");
        gst_bin_add(GST_BIN(pipeline.get()), webrtcbin.get());

        gst_element_set_state(pipeline.get(), GST_STATE_PLAYING);
    }

    void listen() {
        // Here we would listen for incoming messages from the signaling server
        // Deserialize them as JSON and process accordingly
        for (;;) {
            char buf[1024];
            size_t len = socket.read_some(boost::asio::buffer(buf));
            json msg = json::parse(std::string(buf, len));

            if (msg.contains("sdp")) {
                handle_sdp(msg);
            } else if (msg.contains("ice")) {
                handle_ice(msg);
            } else {
                std::cerr << "Unknown message: " << msg.dump() << std::endl;
            }
        }
    }

    void handle_sdp(const json& msg) {
        if (msg["sdp"]["type"] == "answer") {
            std::string sdp_str = msg["sdp"]["sdp"];
            GstSDPMessage* sdp_msg = nullptr;
            GstSdp.sdp_message_parse_buffer(sdp_str.c_str(), sdp_msg);
            GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(
                GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);

            GstPromise* promise = gst_promise_new();
            gst_webrtcbin_emit(webrtcbin.get(), "set-remote-description", answer, promise);
            gst_promise_wait(promise);
        }
    }

    void handle_ice(const json& msg) {
        std::string candidate = msg["ice"]["candidate"];
        int mlineindex = msg["ice"]["sdpMLineIndex"];
        gst_webrtcbin_emit(webrtcbin.get(), "add-ice-candidate", mlineindex, candidate);
    }
};

int main() {
    try {
        io_context io;
        WebRTCSend webrtc(io);

        std::string signaling_server = "ws://0.0.0.0:8765";
        webrtc.connect(signaling_server);
        webrtc.start_pipeline();

        std::thread listener([&]() {
            webrtc.listen();
        });

        listener.join();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
