# API Reference

## Core Components

### VideoSender

#### Class Definition
```cpp
class VideoSender {
public:
    VideoSender();
    ~VideoSender();
    
    // Configuration
    void set_config(const Config& config);
    Config get_config() const;
    
    // Pipeline Control
    void start();
    void stop();
    void pause();
    void resume();
    
    // WebRTC
    void create_offer();
    void handle_answer(const std::string& sdp);
    void add_ice_candidate(const std::string& candidate);
    
    // Status
    Status get_status() const;
    Metrics get_metrics() const;
};
```

#### Methods

##### set_config
```cpp
void VideoSender::set_config(const Config& config)
```
Sets the configuration for the video sender.

Parameters:
- `config`: Configuration object containing settings

##### get_config
```cpp
Config VideoSender::get_config() const
```
Gets the current configuration.

Returns:
- `Config`: Current configuration object

##### start
```cpp
void VideoSender::start()
```
Starts the video sender pipeline.

##### stop
```cpp
void VideoSender::stop()
```
Stops the video sender pipeline.

##### create_offer
```cpp
void VideoSender::create_offer()
```
Creates a WebRTC offer.

##### handle_answer
```cpp
void VideoSender::handle_answer(const std::string& sdp)
```
Handles a WebRTC answer.

Parameters:
- `sdp`: Session Description Protocol string

### VideoReceiver

#### Class Definition
```cpp
class VideoReceiver {
public:
    VideoReceiver();
    ~VideoReceiver();
    
    // Configuration
    void set_config(const Config& config);
    Config get_config() const;
    
    // Pipeline Control
    void start();
    void stop();
    void pause();
    void resume();
    
    // WebRTC
    void handle_offer(const std::string& sdp);
    void create_answer();
    void add_ice_candidate(const std::string& candidate);
    
    // Status
    Status get_status() const;
    Metrics get_metrics() const;
};
```

#### Methods

##### set_config
```cpp
void VideoReceiver::set_config(const Config& config)
```
Sets the configuration for the video receiver.

Parameters:
- `config`: Configuration object containing settings

##### handle_offer
```cpp
void VideoReceiver::handle_offer(const std::string& sdp)
```
Handles a WebRTC offer.

Parameters:
- `sdp`: Session Description Protocol string

##### create_answer
```cpp
void VideoReceiver::create_answer()
```
Creates a WebRTC answer.

### SignalingServer

#### Class Definition
```cpp
class SignalingServer {
public:
    SignalingServer();
    ~SignalingServer();
    
    // Server Control
    void start();
    void stop();
    
    // Client Management
    void add_client(const Client& client);
    void remove_client(const std::string& client_id);
    
    // Message Handling
    void handle_message(const Message& message);
    void broadcast(const Message& message);
    
    // Status
    ServerStatus get_status() const;
};
```

#### Methods

##### start
```cpp
void SignalingServer::start()
```
Starts the signaling server.

##### stop
```cpp
void SignalingServer::stop()
```
Stops the signaling server.

##### handle_message
```cpp
void SignalingServer::handle_message(const Message& message)
```
Handles incoming messages.

Parameters:
- `message`: Message object to handle

## Data Types

### Config
```cpp
struct Config {
    // Video Settings
    int width;
    int height;
    int fps;
    std::string format;
    
    // Encoding Settings
    int bitrate;
    int gop_size;
    std::string preset;
    
    // Network Settings
    std::string stun_server;
    std::string turn_server;
    std::string turn_username;
    std::string turn_password;
};
```

### Status
```cpp
struct Status {
    // Pipeline Status
    bool is_running;
    bool is_paused;
    std::string state;
    
    // Connection Status
    bool is_connected;
    std::string connection_state;
    
    // Error Status
    bool has_error;
    std::string error_message;
};
```

### Metrics
```cpp
struct Metrics {
    // Performance Metrics
    double fps;
    double bitrate;
    double latency;
    
    // Resource Usage
    double cpu_usage;
    double gpu_usage;
    double memory_usage;
    
    // Network Metrics
    double packet_loss;
    double jitter;
    double bandwidth;
};
```

### Message
```cpp
struct Message {
    // Message Type
    enum class Type {
        OFFER,
        ANSWER,
        ICE_CANDIDATE,
        ERROR
    };
    
    Type type;
    std::string sender;
    std::string receiver;
    std::string content;
};
```

## Events

### VideoSender Events
```cpp
class VideoSenderEvents {
public:
    // Pipeline Events
    virtual void on_pipeline_started();
    virtual void on_pipeline_stopped();
    virtual void on_pipeline_error(const std::string& error);
    
    // WebRTC Events
    virtual void on_offer_created(const std::string& sdp);
    virtual void on_ice_candidate(const std::string& candidate);
    virtual void on_connection_state_changed(const std::string& state);
    
    // Status Events
    virtual void on_status_changed(const Status& status);
    virtual void on_metrics_updated(const Metrics& metrics);
};
```

### VideoReceiver Events
```cpp
class VideoReceiverEvents {
public:
    // Pipeline Events
    virtual void on_pipeline_started();
    virtual void on_pipeline_stopped();
    virtual void on_pipeline_error(const std::string& error);
    
    // WebRTC Events
    virtual void on_answer_created(const std::string& sdp);
    virtual void on_ice_candidate(const std::string& candidate);
    virtual void on_connection_state_changed(const std::string& state);
    
    // Status Events
    virtual void on_status_changed(const Status& status);
    virtual void on_metrics_updated(const Metrics& metrics);
};
```

### SignalingServer Events
```cpp
class SignalingServerEvents {
public:
    // Server Events
    virtual void on_server_started();
    virtual void on_server_stopped();
    virtual void on_server_error(const std::string& error);
    
    // Client Events
    virtual void on_client_connected(const std::string& client_id);
    virtual void on_client_disconnected(const std::string& client_id);
    
    // Message Events
    virtual void on_message_received(const Message& message);
    virtual void on_message_sent(const Message& message);
};
```

## Error Handling

### Error Codes
```cpp
enum class ErrorCode {
    // Pipeline Errors
    PIPELINE_CREATION_FAILED,
    PIPELINE_START_FAILED,
    PIPELINE_STOP_FAILED,
    
    // WebRTC Errors
    WEBRTC_INIT_FAILED,
    OFFER_CREATION_FAILED,
    ANSWER_CREATION_FAILED,
    ICE_FAILED,
    
    // Network Errors
    CONNECTION_FAILED,
    SIGNALING_FAILED,
    STUN_FAILED,
    TURN_FAILED
};
```

### Error Class
```cpp
class Error {
public:
    Error(ErrorCode code, const std::string& message);
    
    ErrorCode get_code() const;
    std::string get_message() const;
    std::string to_string() const;
    
private:
    ErrorCode code_;
    std::string message_;
};
```

## Configuration

### Configuration Options
```yaml
video:
  width: 1280
  height: 720
  fps: 30
  format: NV12

encoding:
  bitrate: 4000000
  gop_size: 30
  preset: low-latency

network:
  stun:
    server: stun:stun.l.google.com:19302
  turn:
    server: turn:turn.example.com:3478
    username: user
    password: pass

webrtc:
  ice_timeout: 5000
  gathering_timeout: 5000
```

## Examples

### Basic Usage
```cpp
// Create sender
VideoSender sender;
sender.set_config(config);
sender.start();

// Create receiver
VideoReceiver receiver;
receiver.set_config(config);
receiver.start();

// Create signaling server
SignalingServer server;
server.start();
```

### Event Handling
```cpp
class MySenderEvents : public VideoSenderEvents {
    void on_pipeline_started() override {
        std::cout << "Pipeline started" << std::endl;
    }
    
    void on_pipeline_error(const std::string& error) override {
        std::cerr << "Pipeline error: " << error << std::endl;
    }
};

// Set event handler
sender.set_events(std::make_shared<MySenderEvents>());
```

### Error Handling
```cpp
try {
    sender.start();
} catch (const Error& error) {
    std::cerr << "Error: " << error.to_string() << std::endl;
} 