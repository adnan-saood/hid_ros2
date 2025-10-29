# Controllers

ROS 2 controllers provided by HID ROS 2.

## hid_generic_broadcaster

Generic broadcaster for HID sensor data.

**Features:**
- Publishes all state interfaces
- Type-aware message generation
- Configurable publishing rate

**Topics Published:**
- `/<sensor_name>_states`: State data

## hid_diagnostic_broadcaster

Device health monitoring controller.

**Features:**
- Connection status tracking
- Error rate monitoring
- Diagnostic message publishing

**Topics Published:**
- `/diagnostics`: Diagnostic messages

## signal_command_controller

Example controller demonstrating bidirectional communication.

**Features:**
- Automatically cycles through parameter combinations
- Writes to command interfaces (amplitude, frequency, phase)
- Publishes current parameters to topics
- Example for custom controllers

**Topics Published:**
- `~/current_params`: Current signal parameters (Float64MultiArray)

**Behavior:**
- Automatically updates parameters every 10 seconds (configurable)
- Cycles through predefined amplitude, frequency, and phase combinations
- Demonstrates how to write to command interfaces in ros2_control

See [Custom Controllers](advanced/custom_controllers.md) for creating your own.
