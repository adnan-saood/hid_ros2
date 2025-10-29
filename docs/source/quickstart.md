# Quick Start Guide

This guide will walk you through creating your first HID device integration with ROS 2 in just a few minutes.

For a complete end-to-end tutorial from cloning the repository to creating a custom device, see the [Complete Tutorial](tutorial_custom_device.md).

## Overview

The HID ROS 2 workflow consists of three main steps:

1. **Define** your device schema in YAML
2. **Build** your package (code generation happens automatically)
3. **Launch** your device integration

## Step 1: Create a Package

Create a new ROS 2 package for your HID device:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_hid_device \
  --dependencies rclcpp hardware_interface controller_manager
```

## Step 2: Define Your Device Schema

Create a schema directory and YAML file:

```bash
cd my_hid_device
mkdir schema
```

Create `schema/hid_device.yaml`:

```yaml
# Device identification
device_name: "my_sensor"
vendor_id: "0x046d"      # Your device's USB VID
product_id: "0xc07e"     # Your device's USB PID

# Report IDs
input_report_id: 2       # Device -> Computer
output_report_id: 1      # Computer -> Device (optional)

# ROS 2 configuration
sensor_name: "hid_sensor"
frame_id: "sensor_frame"
update_rate: 250         # Hz

# Data fields - this defines your HID report structure
fields:
  - name: "position_x"
    type: "float32"
    description: "X position in meters"

  - name: "position_y"
    type: "float32"
    description: "Y position in meters"

  - name: "velocity"
    type: "int16"
    description: "Velocity in mm/s"

  - name: "button"
    type: "uint8"
    description: "Button state (0 or 1)"
```

## Step 3: Configure CMake

Edit `CMakeLists.txt` to add code generation:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_hid_device)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hid_descriptor_generator REQUIRED)

# Include the code generator
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

# Generate all files from schema
hid_generate(
  SCHEMA_FILE "schema/hid_device.yaml"
)

# Install schema
install(DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

## Step 4: Build

Build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_hid_device
source install/setup.bash
```

The build process automatically generates:
- ✓ HID descriptor C header for firmware (`include/my_hid_device/hid_descriptor.h`)
- ✓ URDF with state interfaces (`urdf/hid_robot.urdf.xacro`)
- ✓ Controller configuration (`config/controllers.yaml`)
- ✓ Launch file (`launch/hid.launch.py`)

## Step 5: Verify Generated Files

Check what was generated:

```bash
find install/my_hid_device -type f
```

You should see:
```
install/my_hid_device/
├── include/my_hid_device/hid_descriptor.h
├── share/my_hid_device/
│   ├── urdf/hid_robot.urdf.xacro
│   ├── config/controllers.yaml
│   ├── launch/hid.launch.py
│   └── schema/hid_device.yaml
```

## Step 6: Prepare Your Firmware

Use the generated header in your microcontroller firmware:

```c
#include "my_hid_device/hid_descriptor.h"

// USB HID descriptor
const uint8_t* hid_desc = hid_report_descriptor;
uint16_t hid_desc_len = HID_REPORT_DESCRIPTOR_SIZE;

// Your report structure (matches YAML schema)
typedef struct __attribute__((packed)) {
    uint8_t report_id;    // Must be HID_INPUT_REPORT_ID (2)
    float position_x;
    float position_y;
    int16_t velocity;
    uint8_t button;
} SensorReport;

// In your main loop
void send_sensor_data(void) {
    SensorReport report = {
        .report_id = HID_INPUT_REPORT_ID,
        .position_x = get_x_position(),
        .position_y = get_y_position(),
        .velocity = get_velocity(),
        .button = read_button()
    };

    hid_send((uint8_t*)&report, sizeof(report));
}
```

## Step 7: Launch Your Device

Connect your HID device and launch:

```bash
ros2 launch my_hid_device hid.launch.py
```

You should see:
```
[INFO] [controller_manager]: Loading controller 'hid_generic_broadcaster'
[INFO] [controller_manager]: Configured and activated 'hid_generic_broadcaster'
[INFO] [hid_hardware]: Successfully opened HID device 046d:c07e
[INFO] [hid_hardware]: Device connected and ready
```

## Step 8: Monitor Data

Check available interfaces:

```bash
ros2 control list_hardware_interfaces
```

Output:
```
command interfaces
state interfaces
    hid_sensor/position_x [available] [claimed]
    hid_sensor/position_y [available] [claimed]
    hid_sensor/velocity [available] [claimed]
    hid_sensor/button [available] [claimed]
```

Monitor the published data:

```bash
ros2 topic echo /hid_sensor_states
```

## Debugging Your Device

### List Connected HID Devices

```bash
ros2 run hid_tools inspect_device --list
```

### Monitor Raw USB Reports

```bash
ros2 run hid_tools inspect_device --vid 0x046d --pid 0xc07e
```

This shows you exactly what your device is sending.

### Validate Your Schema

```bash
ros2 run hid_tools validate_schema schema/hid_device.yaml --strict
```

## Next Steps

Now that you have a working integration:

1. **Customize Controllers**: See [Controllers Guide](controllers.md)
2. **Add Bidirectional Communication**: See [Schema Reference](schema_reference.md) for output reports
3. **Create Custom Controllers**: See [Custom Controllers](advanced/custom_controllers.md)
4. **Integrate with Your Robot**: Use the state interfaces in your robot's control system

## Common Issues

### Device Not Found

Check device is connected and has correct VID/PID:

```bash
lsusb
ros2 run hid_tools inspect_device --list
```

### Permission Denied

Set up udev rules (see [Installation](installation.md#setting-up-usb-permissions)).

### Wrong Data Values

Use `inspect_device` to compare raw USB data with ROS 2 topics:

```bash
# Terminal 1: Raw USB data
ros2 run hid_tools inspect_device --vid 0x046d --pid 0xc07e

# Terminal 2: ROS 2 topic
ros2 topic echo /hid_sensor_states
```

### Build Errors

Ensure code generator is built first:

```bash
colcon build --packages-select hid_descriptor_generator
colcon build --packages-select my_hid_device
```

## Complete Example

See the `examples/dummy_pose_device` package for a complete working example with:
- Full YAML schema
- Simulated HID device
- Custom controller
- Launch files
