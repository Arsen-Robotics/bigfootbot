# motor_control

This package handles the kinematics and communication with the RoboClaw motor controller.

## Nodes

### `roboclaw_control_node`

Primary interface to the physical RoboClaw hardware.

- **Subscribes**:
  - `/cmd_vel_out` (`geometry_msgs/Twist`): Prioritized velocity command from `twist_mux`.
- **Publishes**:
  - `/roboclaw_state` (`bfb_interfaces/RoboclawState`): Detailed motor status (currents, voltage, temperature).
  - `/battery_state` (`sensor_msgs/BatteryState`): Estimated battery percentage and current voltage.
  - `/wheel_speed` (`std_msgs/Float32`): Real-time speed in km/h.
- **Parameters**:
  - `wheel_track`: 0.65m
  - `wheel_diameter`: 0.33m
  - `max_rpm`: 177.5
  - `max_motor_current`: 30A (automatic stop threshold)

### `joy_to_twist_node`

Converts raw joystick inputs into velocity commands.

- **Subscribes**:
  - `/joy` (`sensor_msgs/Joy`): Raw inputs from the physical controller.
- **Publishes**:
  - `/joy_vel` (`geometry_msgs/Twist`): Manual control velocity.
  - `/arduino_gateway` (`std_msgs/String`): Commands for the Arduino (lights, buzzer, etc.).
- **Parameters**:
  - Configurable mappings for PS3 and Logitech controllers (see `config/params.yaml`).

## Configuration

- `**config/params.yaml`**: Hardware constants and node-specific mappings.
- `**config/twist_mux.yaml**`: Priority settings for different velocity sources.
- `**config/twist_mux.yaml**`: Priority levels:
  - `joy_vel`: 10 (Highest)
  - `nav_vel`: 5 (Autonomous)

## Hardware Integration

- **Device**: `/dev/roboclaw` (symlink created via udev).
- **Permissions**: Ensure the user is in the `dialout` group or use the provided udev rules in `udev/99-roboclaw.rules`.

