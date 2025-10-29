# HID ROS2 - Quick Start Guide

## Overview

This workspace provides a seamless way to integrate HID devices with ROS2 using a **single YAML schema** as the source of truth.

## Workflow

### 1. Define Your Device Schema

Create `schema/hid_device.yaml` in your bringup package:

```yaml
# Device metadata
device_name: "my_hid_device"
vendor_id: "0x046d"      # Your device's USB VID
product_id: "0xc07e"     # Your device's USB PID
report_id: 1

# ROS2 parameters
sensor_name: "hid_input"
frame_id: "hid_frame"
update_rate: 250         # Hz

# HID Report Structure
# This defines what your device sends
fields:
  - name: "x"
    type: "uint8"
    description: "X position"

  - name: "y"
    type: "uint8"
    description: "Y position"

  - name: "yaw"
    type: "int16"
    description: "Yaw angle"

  - name: "buttons"
    type: "uint8"
    count: 2
    description: "Button states"
```

### 2. Add Dependencies

In your `package.xml`:
```xml
<build_depend>hid_descriptor_generator</build_depend>
```

In your `CMakeLists.txt`:
```cmake
find_package(hid_descriptor_generator REQUIRED)
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

hid_generate(
  SCHEMA_FILE "schema/hid_device.yaml"
)

install(DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)
```

### 3. Build

```bash
cd /home/adnan/dev/hid_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select your_package
```

**That's it!** The build process automatically generates:
- ✅ HID report descriptor (C header)
- ✅ URDF with correct state interfaces
- ✅ Controller configuration
- ✅ Launch file with TF broadcaster

### 4. Launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch your_package hid.launch.py
```

## What Gets Generated

### C Header (`include/<package>/hid_descriptor.h`)

Use this in your microcontroller firmware:

```c
#include "your_package/hid_descriptor.h"

// In your USB HID init:
const uint8_t* desc = HID_REPORT_DESCRIPTOR;
uint16_t desc_len = HID_REPORT_DESCRIPTOR_LEN;

// Your report structure:
typedef struct __attribute__((packed)) {
    uint8_t report_id;  // = HID_REPORT_ID
    uint8_t x;
    uint8_t y;
    int16_t yaw;
    uint8_t buttons[2];
} my_hid_report_t;
```

### URDF (`urdf/hid_robot.urdf.xacro`)

Defines state interfaces matching your schema:
- `hid_input/report_id`
- `hid_input/x`
- `hid_input/y`
- `hid_input/yaw`
- `hid_input/buttons_0`
- `hid_input/buttons_1`

### Launch File (`launch/hid.launch.py`)

Ready-to-use launch file with:
- ros2_control node
- Controller spawners
- Static TF (world → hid_frame)

## Example: Current dummy_bringup

Check `/home/adnan/dev/hid_ros2/dummy_bringup/schema/hid_device.yaml` for a working example.

Generated files are in:
```
build/dummy_bringup/generated/
├── include/dummy_bringup/hid_descriptor.h
├── urdf/hid_robot.urdf.xacro
├── config/controllers.yaml
└── launch/hid.launch.py
```

Installed files go to:
```
install/dummy_bringup/share/dummy_bringup/
├── urdf/
├── config/
├── launch/
└── schema/
```

## Supported Types

| Type | Bits | Signed | Use Case |
|------|------|--------|----------|
| `bool` | 1 | No | Flags, buttons |
| `int8` | 8 | Yes | Small signed values |
| `uint8` | 8 | No | Bytes, small unsigned |
| `int16` | 16 | Yes | Sensor readings |
| `uint16` | 16 | No | Larger unsigned values |
| `int32` | 32 | Yes | Large signed values |
| `uint32` | 32 | No | Large unsigned values |
| `float32` | 32 | Yes | Floating point (limited HID support) |
| `float64` | 64 | Yes | Double precision (limited HID support) |

## Arrays

Use `count` for fixed-size arrays:

```yaml
- name: "accel"
  type: "int16"
  count: 3  # Creates accel_0, accel_1, accel_2
```

## Custom Controllers

The generated files work with any controller. Edit the controller manually:

```cpp
// In your controller's update():
double x = state_interfaces_[0].get_value();        // hid_input/x
double y = state_interfaces_[1].get_value();        // hid_input/y
double yaw = state_interfaces_[2].get_value();      // hid_input/yaw
double btn0 = state_interfaces_[3].get_value();     // hid_input/buttons_0
double btn1 = state_interfaces_[4].get_value();     // hid_input/buttons_1
```

## Debugging

### Check generated files:
```bash
find build/your_package/generated -type f
```

### View HID descriptor:
```bash
cat install/your_package/include/your_package/hid_descriptor.h
```

### Check state interfaces:
```bash
ros2 control list_hardware_interfaces
```

### Monitor values:
```bash
ros2 topic echo /hid_input_states  # If using joint_state_broadcaster
```

## Tips

1. **Match firmware and schema**: Keep your YAML schema in version control alongside firmware
2. **Update rate**: Higher rates (500-1000 Hz) for gaming mice, lower (100-250 Hz) for sensors
3. **Frame ID**: Use descriptive names like `mouse_frame`, `joystick_frame`, etc.
4. **Regeneration**: Files regenerate automatically when you modify the YAML and rebuild


## Troubleshooting

**Build fails:**
- Ensure `hid_descriptor_generator` is built first
- Source both ROS and workspace setup scripts

**Device not connecting:**
- Check vendor_id and product_id match your device
- Verify udev rules for permissions
- Test with `lsusb` to confirm device enumeration

**Wrong interfaces:**
- Verify YAML schema matches device report structure
- Check firmware packing/alignment with `__attribute__((packed))`
