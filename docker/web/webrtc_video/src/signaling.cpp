// Required WebSocket++ headers
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <set>
#include <string>
#include <jsoncpp/json/json.h>

// Define server type using WebSocket++ ASIO config
typedef websocketpp::server<websocketpp::config::asio> server;

/**
 * Signaling server class that handles WebSocket connections and message forwarding
 * between WebRTC peers
 */
class signaling_server {
public:
    // Constructor - initializes the WebSocket server and sets up handlers
    signaling_server() {
        m_server.init_asio();
        m_server.set_open_handler(std::bind(&signaling_server::on_open, this, std::placeholders::_1));
        m_server.set_close_handler(std::bind(&signaling_server::on_close, this, std::placeholders::_1));
        m_server.set_message_handler(std::bind(&signaling_server::on_message, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * Starts the signaling server on the specified port
     * @param port Port number to listen on
     */
    void run(uint16_t port) {
        std::cout << "Starting signaling server on port " << port << std::endl;
        m_server.listen(port);
        m_server.start_accept();
        m_server.run();
    }

private:
    /**
     * Handler called when a new WebSocket connection is established
     * @param hdl Connection handle for the new client
     */
    void on_open(websocketpp::connection_hdl hdl) {
        m_connections.insert(hdl);
        std::cout << "Client connected" << std::endl;
    }

    /**
     * Handler called when a WebSocket connection is closed
     * @param hdl Connection handle for the disconnected client
     */
    void on_close(websocketpp::connection_hdl hdl) {
        m_connections.erase(hdl);
        std::cout << "Client disconnected" << std::endl;
    }

    /**
     * Handler for incoming WebSocket messages
     * Parses JSON messages and forwards them to other connected clients
     * @param hdl Connection handle of the sending client
     * @param msg Pointer to the received message
     */
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        // Parse incoming JSON message
        std::string payload = msg->get_payload();
        Json::Value root;
        Json::Reader reader;

        if (!reader.parse(payload, root)) {
            std::cerr << "Failed to parse JSON message: " << reader.getFormattedErrorMessages() << std::endl;
            return;
        }

        // Ensure we're sending a text message (opcode 0x1)
        if (msg->get_opcode() == websocketpp::frame::opcode::text) {
            // Forward the message to all other clients
            for (auto it : m_connections) {
                if (it.lock() != hdl.lock()) {
                    m_server.send(it, payload, msg->get_opcode());
                }
            }
        }
    }

    server m_server;  // WebSocket server instance
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> m_connections;  // Set of connected clients
};

int main() {
    signaling_server server_instance;
    server_instance.run(8765); // Run server on port 8765
    return 0;
}