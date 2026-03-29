# System Architecture

The BigfootBot is a modular, ROS 2-based autonomous robot designed to work on a distributed compute stack (NVIDIA Jetson, Raspberry Pi, Arduino).

---

## 1. System Context Diagram (Level 1)

This high-level view shows how the robot interacts with the remote operator and the physical world. It omits internal ROS 2 details to focus on the overall system boundaries.

```mermaid
graph TD
    subgraph "Remote Infrastructure"
        OP[Remote Operator Interface]
        SIG[WebRTC Signaling Server]
    end

    subgraph "BigfootBot (Physical Robot)"
        subgraph "On-Board Compute"
            JET[NVIDIA Jetson: Perception Machine]
            RPI[Raspberry Pi: Control Hub]
        end
        
        HW[Physical Hardware: Motors & Sensors]
    end

    %% High-level Connections
    OP <== "WebRTC (Video/Telemetry)" ==> JET
    OP <== "Websocket (Commands/Status)" ==> RPI
    SIG -. "Signaling/Handshake" .-> JET
    SIG -. "Signaling/Handshake" .-> OP
    
    JET <== "LAN (ROS 2 Network)" ==> RPI
    
    RPI <== "Serial / USB / GPIO" ==> HW
    JET <== "USB (Camera/GPS)" ==> HW

    %% STYLING
    style JET fill:#76b900,stroke:#333,color:#fff
    style RPI fill:#c51a4a,stroke:#333,color:#fff
    style OP fill:#fff,stroke:#333
    style HW fill:#eee,stroke:#333
```

---

## 2. Technical Context & Drill-Down

To understand the internal logic, communication, and deployment of the system, please refer to the following documents:

| Level | Document | Audience | Purpose |
| :--- | :--- | :--- | :--- |
| **System** | [**README.md**](./README.md) | Anyone | High-level system context and boundaries. |
| **Logical** | [**Node Graph (Logical View)**](./node_graph.md#1-logical-view-ros-2-node-graph) | ROS Developers | Detailed ROS 2 topics, services, and node-to-node communication. |
| **Physical** | [**Deployment Map (Physical View)**](./node_graph.md#2-physical-view-deployment-map) | DevOps / Hardware | Container-to-machine mapping and networking. |
| **Subsystem** | [**Control System Architecture**](./control/README.md) | Control Engineers | Functional logic for velocity arbitration and safety. |

---

## Directory Structure Overview
- **`/src`**: Individual ROS 2 packages for perception, control, and interfaces.
- **`/docs`**: High-level architectural, decision, and infrastructure documentation.
- **`/docker`**: Containerization stacks (Jetson and Raspberry Pi specific).
- **`/webrtc-browser`**: Modern TypeScript/React-based remote operator dashboard.
