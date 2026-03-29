# Control System Architecture

This document describes the high-level control flow of the BigfootBot, from user/autonomous input to physical motor movement.

## System Overview

The control system follows a modular architecture where multiple input sources (Manual and Autonomous) are prioritized by a multiplexer before being executed by the motor controller.

A detailed mapping of all ROS 2 nodes and their physical deployment can be found in the [**Logical & Physical Views**](../node_graph.md).

```mermaid
graph TD
    subgraph "NVIDIA Jetson (Perception Machine)"
        RF[Road Follower Node] -- "/nav_vel (Priority 5)" --> MUX
    end

    subgraph "Raspberry Pi (Control Hub)"
        JOY[Joy to Twist Node] -- "/joy_vel (Priority 10)" --> MUX
        MUX{twist_mux} -- "/cmd_vel_out" --> RC[RoboClaw Control Node]
        RC -- "Relay Service" --> AG[Arduino Gateway]
    end

    subgraph "Physical Hardware"
        RC -- "USB Serial" --> DRIVER[RoboClaw 2x15A]
        DRIVER -- "PWM" --> MOTORS[DC Motors]
        AG -- "Serial" --> ARDUINO[Arduino Mega]
    end

    %% Link across machines
    RF -. "Network Link (ROS 2)" .-> MUX

    style MUX fill:#f96,stroke:#333
    style RC fill:#bbf,stroke:#333
    style DRIVER fill:#dfd,stroke:#333
    style RF fill:#ff9,stroke:#333
```

## Core Components

### 1. Input Sources
*   **Manual Control**: A joystick (PS3 or Logitech) is processed by the `joy_to_twist_node`, publishing to the high-priority `/joy_vel` topic.
*   **Autonomous Navigation**: Vision-based road following or Nav2 stacks publish to the lower-priority `/nav_vel` topic.

### 2. Multiplexer (`twist_mux`)
The `twist_mux` node ensures safety by prioritizing manual overrides. If a message is received on `/joy_vel`, it will override any autonomous commands on `/nav_vel` for the duration of the timeout (0.5s).

### 3. Execution (`motor_control`)
The `roboclaw_control_node` is the final stage of the software stack. It performs differential drive kinematics to convert linear and angular velocity into motor-specific duty cycles.

## Safety Features
*   **Joystick Deadman Switch**: The `joy_to_twist_node` requires a specific "enable" axis to be held.
*   **Overcurrent Protection**: The `roboclaw_control_node` monitors motor current and will automatically stop the robot if it exceeds 30A.
*   **Battery Monitoring**: Real-time voltage monitoring ensures the robot stops before the Li-ion battery is damaged.
