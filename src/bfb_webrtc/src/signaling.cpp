// Required WebSocket++ headers
#include <rclcpp/rclcpp.hpp>
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
class SignalingNode : public rclcpp::Node {
public:
    /**
     * @brief Constructs a signaling server instance.
     * 
     * Initializes the WebSocket server and sets up connection handlers.
     * 
     * @throws std::runtime_error if server initialization fails
     */
    SignalingNode() : Node("signaling_node") {
        int port = this->declare_parameter<int>("port", 8765);
        try {
            m_server.init_asio(); // Initialize the WebSocket server
            m_server.set_open_handler(std::bind(&SignalingNode::onOpen, this, std::placeholders::_1));
            m_server.set_close_handler(std::bind(&SignalingNode::onClose, this, std::placeholders::_1));
            m_server.set_message_handler(std::bind(&SignalingNode::onMessage, this, std::placeholders::_1, std::placeholders::_2));
            m_server.listen(port);
            m_server.start_accept();
            // Run server in a separate thread so ROS 2 can spin
            m_server_thread = std::thread([this]() { m_server.run(); });
            RCLCPP_INFO(this->get_logger(), "Signaling server started.");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize signaling server: %s", e.what());
            throw;
        }
    }

    ~SignalingNode() {
        m_server.stop();
        if (m_server_thread.joinable()) m_server_thread.join();
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
            RCLCPP_INFO(this->get_logger(), "Client connected. Total connections: %zu", m_connections.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in onOpen: %s", e.what());
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
            RCLCPP_INFO(this->get_logger(), "Client disconnected. Remaining connections: %zu", m_connections.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in onClose: %s", e.what());
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
            std::string payload = message->get_payload();
            Json::Value root;
            Json::Reader reader;

            if (!reader.parse(payload, root)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON message: %s", reader.getFormattedErrorMessages().c_str());
                return;
            }

            if (message->get_opcode() == websocketpp::frame::opcode::text) {
                for (const auto& conn : m_connections) {
                    if (conn.lock() != connectionHandle.lock()) {
                        try {
                            m_server.send(conn, payload, message->get_opcode());
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(this->get_logger(), "Error sending message to client: %s", e.what());
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in onMessage: %s", e.what());
        }
    }

    WebSocketServer m_server;
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> m_connections;
    std::thread m_server_thread;
};

/**
 * @brief Main function to run the signaling server.
 * 
 * Initializes the signaling server and starts it on port 8765.
 * 
 * @return 0 on success, 1 on failure
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SignalingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}