#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <jsoncpp/json/json.h>

typedef websocketpp::client<websocketpp::config::asio_client> client;

void on_open(websocketpp::connection_hdl hdl, client* c) {
    std::cout << "Connected! Sending message..." << std::endl;

    // Create a JSON object
    Json::Value msg;
    msg["type"] = "offer";  // Example signaling message
    msg["sdp"] = "SDP data here";  // SDP or other data

    // Convert to string
    Json::StreamWriterBuilder writer;
    std::string message = Json::writeString(writer, msg);

    // Send the JSON message
    c->send(hdl, message, websocketpp::frame::opcode::text);
}

int main() {
    client c;

    c.init_asio();
    c.set_open_handler(std::bind(&on_open, std::placeholders::_1, &c));

    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection("ws://localhost:8765", ec);

    if (ec) {
        std::cout << "Connection error: " << ec.message() << std::endl;
        return 1;
    }

    c.connect(con);
    c.run();

    return 0;
}
