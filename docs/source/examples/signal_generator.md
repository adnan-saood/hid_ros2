# Signal Generator Example

Example demonstrating bidirectional communication with automatic parameter updates.

## Overview

The signal_generator example shows:
- Input and output reports
- Command interface usage
- Automatic parameter cycling
- Bidirectional data flow

## Schema

Features both input and output reports:

```yaml
device_name: "Signal Generator"
vendor_id: "0xCAFE"
product_id: "0x4000"

input_report_id: 2   # Device → PC
output_report_id: 1  # PC → Device

sensor_name: "signal_gen"
gpio_name: "signal_params"
frame_id: "signal_gen_frame"
update_rate: 250
publish_rate: 100.0

# Input fields (device sends to PC)
inputs:
  - name: "timestamp"
    type: "float32"
  - name: "result"
    type: "float32"

# Output fields (PC sends to device)
outputs:
  - name: "amplitude"
    type: "float32"
  - name: "frequency"
    type: "float32"
  - name: "phase"
    type: "float32"
```

## How It Works

The signal generator demonstrates bidirectional communication:

1. **Device (firmware)** receives parameters (amplitude, frequency, phase) via output report
2. **Device** computes: `result = amplitude * sin(frequency * t + phase)`
3. **Device** sends result via input report
4. **Controller** automatically cycles through different parameter combinations every 10 seconds
5. **Controller** publishes current parameters to `~/current_params` topic

## Building

```bash
colcon build --packages-select signal_command_controller signal_generator_bringup
```

## Running

```bash
ros2 launch signal_generator_bringup signal_gen.launch.py
```

## Monitoring

### Monitor Generated Signal

```bash
# Watch the signal output from device
ros2 topic echo /signal_gen_broadcaster/signal_gen_states
```

### Monitor Current Parameters

```bash
# Watch the parameters being sent to device
ros2 topic echo /signal_command_controller/current_params
```

The controller automatically cycles through parameter combinations:
- **Amplitude**: 0.5, 1.0, 1.5, 2.0
- **Frequency**: 0.5, 1.0, 2.0, 3.0 Hz
- **Phase**: 0, π/4, π/2, π

Parameters update every 10 seconds (configurable via `update_interval` parameter).

## Source Code

See the example package for implementation.
