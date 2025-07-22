# Signal Generator Bringup

Bidirectional HID device demonstration: PC sends signal parameters (amplitude, frequency, phase) to the device, device calculates and returns result = amplitude * sin(frequency * t + phase).

## Architecture - Single Source of Truth

This package follows the schema-driven pattern where everything is generated from the YAML schema.

## Workflow

1. Define device in schema/signal_generator.yaml
2. Build package - files auto-generated from schema
3. Optionally customize config/signal_gen_controllers.yaml for controller parameters
4. Optionally customize launch/signal_gen.launch.py to choose which controllers to spawn

## Device Interfaces

State Interfaces (device to PC):
- signal_gen/report_id
- signal_gen/timestamp
- signal_gen/result

Command Interfaces (PC to device):
- signal_params/amplitude
- signal_params/frequency
- signal_params/phase

## Controllers

signal_command_controller: Auto-cycles through parameter values every 10 seconds
hid_generic_broadcaster: Reads device state and publishes to topic
hid_diagnostic_broadcaster: Optional device health monitoring

## Usage

Build:
  colcon build --packages-select signal_generator_bringup signal_command_controller

Launch:
  ros2 launch signal_generator_bringup signal_gen.launch.py

Monitor:
  ros2 topic echo /signal_command_controller/current_params
  ros2 topic echo /hid_generic_broadcaster/hid_state
