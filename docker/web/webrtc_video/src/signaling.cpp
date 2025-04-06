// Required WebSocket++ headers
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <set>
#include <string>
#include <jsoncpp/json/json.h>
#include <memory>
#include <stdexcept>
#include <system_error>

// Define server type using WebSocket++ ASIO config
using WebSocketServer = websocketpp::server<websocketpp::config::asio>;

/**
 * @brief Signaling server class that handles WebSocket connections and message forwarding
 * between WebRTC peers.
 * 
 * This class implements a WebSocket server that facilitates WebRTC signaling by
 * forwarding SDP offers/answers and ICE candidates between peers.
 */
class SignalingServer {
public:
    /**
     * @brief Constructs a signaling server instance.
     * 
     * Initializes the WebSocket server and sets up connection handlers.
     * 
     * @throws std::runtime_error if server initialization fails
     */
    SignalingServer() {
        try {
            m_server.init_asio(); // Initialize the WebSocket server
            m_server.set_open_handler(std::bind(&SignalingServer::onOpen, this, std::placeholders::_1));
            m_server.set_close_handler(std::bind(&SignalingServer::onClose, this, std::placeholders::_1));
            m_server.set_message_handler(std::bind(&SignalingServer::onMessage, this, std::placeholders::_1, std::placeholders::_2));
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to initialize signaling server: " + std::string(e.what()));
        }
    }

    /**
     * @brief Starts the signaling server on the specified port.
     * 
     * @param port Port number to listen on
     * @throws std::runtime_error if server fails to start
     */
    void run(uint16_t port) {
        try {
            std::cout << "Starting signaling server on port " << port << std::endl;
            m_server.listen(port);
            m_server.start_accept();
            m_server.run();
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to start signaling server: " + std::string(e.what()));
        }
    }

    /**
     * @brief Stops the signaling server gracefully.
     */
    /*void stop() {
        try {
            m_server.stop_listening();
            for (auto& conn : m_connections) {
                m_server.close(conn, websocketpp::close::status::going_away, "Server shutdown");
            }
            m_server.stop();
        } catch (const std::exception& e) {
            std::cerr << "Error during server shutdown: " << e.what() << std::endl;
        }
    }*/

private:
    /**
     * @brief Handler called when a new WebSocket connection is established.
     * 
     * Adds the new client to the set of connected clients and logs the connection.
     * @param connectionHandle Connection handle for the new client
     */
    void onOpen(websocketpp::connection_hdl connectionHandle) {
        try {
            m_connections.insert(connectionHandle);
            std::cout << "Client connected. Total connections: " << m_connections.size() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error in onOpen: " << e.what() << std::endl;
        }
    }

    /**
     * @brief Handler called when a WebSocket connection is closed.
     * 
     * Removes the disconnected client from the set of connected clients and logs the disconnection.
     * @param connectionHandle Connection handle for the disconnected client
     */
    void onClose(websocketpp::connection_hdl connectionHandle) {
        try {
            m_connections.erase(connectionHandle);
            std::cout << "Client disconnected. Remaining connections: " << m_connections.size() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error in onClose: " << e.what() << std::endl;
        }
    }

    /**
     * @brief Handler for incoming WebSocket messages.
     * 
     * Parses JSON messages and forwards them to other connected clients.
     * 
     * @param connectionHandle Connection handle of the sending client
     * @param message Pointer to the received message
     */
    void onMessage(websocketpp::connection_hdl connectionHandle, WebSocketServer::message_ptr message) {
        try {
            // Parse incoming JSON message
            std::string payload = message->get_payload();
            Json::Value root;
            Json::Reader reader;

            if (!reader.parse(payload, root)) {
                std::cerr << "Failed to parse JSON message: " << reader.getFormattedErrorMessages() << std::endl;
                return;
            }

            // Ensure we're sending a text message (opcode 0x1)
            if (message->get_opcode() == websocketpp::frame::opcode::text) {
                // Forward the message to all other clients
                for (const auto& conn : m_connections) {
                    if (conn.lock() != connectionHandle.lock()) {
                        try {
                            m_server.send(conn, payload, message->get_opcode());
                        } catch (const std::exception& e) {
                            std::cerr << "Error sending message to client: " << e.what() << std::endl;
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in onMessage: " << e.what() << std::endl;
        }
    }

    WebSocketServer m_server;  // WebSocket server instance
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> m_connections;  // Set of connected clients
};

/**
 * @brief Main function to run the signaling server.
 * 
 * Initializes the signaling server and starts it on port 8765.
 * 
 * @return 0 on success, 1 on failure
 */
int main() {
    try {
        SignalingServer serverInstance;
        serverInstance.run(8765);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}