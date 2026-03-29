# System Architecture

The BigfootBot is a modular, ROS 2-based autonomous robot designed to work on a distributed compute stack (Jetson, Raspberry Pi, Arduino).

## High-Level System Diagram

The following Mermaid diagram shows how data flows between the major components of the robot across different physical devices.

```mermaid
graph TD
    %% Define Nodes and Hardware Boundaries
    subgraph "Jetson Xavier NX (Master Brain)"
        RS[RealSense D435i] -- "/color/image_raw" --> RF[Road Follower Node]
        RF -- "/nav_vel (Priority 5)" --> TM[Twist Mux]
        JOY[Joystick Node] -- "/joy_vel (Priority 10)" --> TM
        TM -- "/cmd_vel_out" --> RC[RoboClaw Control]
        
        %% Streaming Logic
        RS -- "/image_raw" --> WRTCS[WebRTC Sender Node]
    end

    subgraph "Arduino Mega (Hardware Gateway)"
        RC -- "RelayControl Service" --> AG[Arduino Gateway]
        AG -- "Serial Command" --> ARDUINO[Physical Arduino Mega]
        ARDUINO -- "Electrical Signals" --> HARDWARE[Relays, Buzzer, Lights]
    end

    subgraph "Remote Operator (Web Interface)"
        WRTCS -- "WebRTC Stream (Video)" --> WRTCR[WebRTC Receiver]
        SIG[Signaling Server] <--> WRTCS
        SIG <--> WRTCR
        WRTCR -- "Web Interface" --> UI[User Dashboard]
    end

    subgraph "Physical Hardware"
        RC -- "USB Serial" --> MOTOR_DRIVER[RoboClaw 2x15A]
        MOTOR_DRIVER -- "PWM" --> MOTORS[DC Drive Motors]
    end

    %% Styles for clarity
    style RS fill:#f96,stroke:#333
    style MOTORS fill:#9f9,stroke:#333
    style UI fill:#69f,stroke:#333
    style SIG fill:#f9f,stroke:#333
    style RC fill:#bbf,stroke:#333
```

## Directory Structure Overview
- **`/src`**: Individual ROS 2 packages for perception, control, and interfaces.
- **`/docs`**: High-level architectural, decision, and infrastructure documentation.
- **`/docker`**: Containerization stacks (Master, Teleop, Web, Navigation).
- **`/webrtc-browser`**: Modern TypeScript/React-based remote operator dashboard.

## Technical Context
For more specific details, refer to:
- [**Logical Node Graph**](./node_graph.md)
- [**Control System Architecture**](./control/README.md)
- [**Infrastructure & Networking**](../infra/README.md)
- [**Package: motor_control**](../../src/motor_control/README.md)
