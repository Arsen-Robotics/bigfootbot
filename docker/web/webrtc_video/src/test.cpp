#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <jsoncpp/json/json.h>
#include <iostream>

typedef websocketpp::client<websocketpp::config::asio_client> client;

void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
    std::cout << "Received: " << msg->get_payload() << std::endl;
}

void on_open(websocketpp::connection_hdl hdl) {
    std::cout << "Connection opened, sending message..." << std::endl;
    
    std::string message = "test msg";
    Json::StreamWriterBuilder writer;
    std::string json_message = Json::writeString(writer, message);
    
    // Sending the message
    client* c = nullptr;
    c = reinterpret_cast<client*>(hdl.lock().get());  // Retrieve the client object from the connection handle
    c->send(hdl, json_message, websocketpp::frame::opcode::text);
}

int main() {
    client c;
    std::string uri = "ws://localhost:8765";

    try {
        c.init_asio();
        c.set_message_handler(&on_message);
        c.set_open_handler(&on_open);  // Set the open handler

        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        if (ec) {
            std::cerr << "Connection error: " << ec.message() << std::endl;
            return 1;
        }

        c.run();  // Run the event loop (instead of run_one)
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
