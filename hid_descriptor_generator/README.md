# HID Descriptor Generator

A build-time code generator for ROS2 that creates HID descriptors, URDF files, controller configurations, and launch files from a single YAML schema.

## Overview

This package provides a CMake function `hid_generate()` that automatically generates all necessary files for a HID device integration from a simple YAML schema file.

## Features

- **Single Source of Truth**: Define your HID device structure once in YAML
- **Automatic Code Generation**: Generates C headers, URDF, controller YAML, and launch files
- **Build-Time Integration**: Seamlessly integrates with `colcon build`
- **Type-Safe**: Supports standard C types with proper HID descriptor generation

## Usage

### 1. Add to your package dependencies

In your `package.xml`:
```xml
<build_depend>hid_descriptor_generator</build_depend>
```

### 2. Add to your CMakeLists.txt

```cmake
find_package(hid_descriptor_generator REQUIRED)
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

hid_generate(
  SCHEMA_FILE "schema/my_device.yaml"
)
```

### 3. Create a YAML schema

Create `schema/my_device.yaml`:
```yaml
# Device metadata
device_name: "my_hid_device"
vendor_id: "0x046d"
product_id: "0xc07e"
report_id: 1

# ROS2 parameters
sensor_name: "hid_input"
frame_id: "hid_frame"
update_rate: 250

# HID Report Structure
fields:
  - name: "x"
    type: "int16"
    description: "X position"

  - name: "y"
    type: "int16"
    description: "Y position"

  - name: "buttons"
    type: "uint8"
    count: 4
    description: "Button states"
```

### 4. Build

```bash
colcon build --packages-select your_package
source install/setup.bash
```

### 5. Launch

```bash
ros2 launch your_package hid.launch.py
```

## Supported Types

- `bool` - 1 bit
- `int8`, `uint8` - 8 bits
- `int16`, `uint16` - 16 bits
- `int32`, `uint32` - 32 bits
- `float32` - 32 bits
- `float64` - 64 bits

## Generated Files

The generator creates:

1. **C Header** (`include/<package>/hid_descriptor.h`)
   - HID report descriptor bytes array
   - Constants for report size, report ID, etc.
   - Field offset documentation

2. **URDF** (`urdf/hid_robot.urdf.xacro`)
   - ros2_control hardware plugin configuration
   - State interfaces matching your schema fields
   - Device vendor/product IDs

3. **Controller YAML** (`config/controllers.yaml`)
   - controller_manager configuration
   - Pre-configured broadcasters

4. **Launch File** (`launch/hid.launch.py`)
   - ros2_control node
   - Controller spawners
   - Static TF publisher (world → hid_frame)

## Schema Reference

### Required Fields

- `fields`: List of HID report fields

### Optional Metadata

- `device_name`: Device identifier (default: "hid_device")
- `vendor_id`: USB vendor ID in hex (default: "0xCAFE")
- `product_id`: USB product ID in hex (default: "0x4000")
- `report_id`: HID report ID 0-255 (default: 1)
- `sensor_name`: ros2_control sensor name (default: "hid_input")
- `frame_id`: TF frame name (default: "hid_frame")
- `update_rate`: Controller update rate in Hz (default: 250)

### Field Definition

```yaml
- name: "field_name"       # Required: interface name in ros2_control
  type: "uint8"            # Required: data type
  count: 1                 # Optional: array size (default: 1)
  description: "..."       # Optional: documentation
```

## Example: Mouse

```yaml
device_name: "gaming_mouse"
vendor_id: "0x046d"
product_id: "0xc07e"
report_id: 1
sensor_name: "mouse_input"
frame_id: "mouse_frame"
update_rate: 1000

fields:
  - name: "dx"
    type: "int16"
    description: "X movement delta"

  - name: "dy"
    type: "int16"
    description: "Y movement delta"

  - name: "wheel"
    type: "int8"
    description: "Scroll wheel delta"

  - name: "buttons"
    type: "uint8"
    count: 2
    description: "Button states (16 buttons max)"
```

## Using in Firmware

The generated C header can be used directly in your microcontroller firmware:

```c
#include "your_package/hid_descriptor.h"

// Use in USB HID configuration
const uint8_t* get_hid_descriptor(void) {
    return HID_REPORT_DESCRIPTOR;
}

uint16_t get_hid_descriptor_len(void) {
    return HID_REPORT_DESCRIPTOR_LEN;
}

// Your report structure should match the schema
typedef struct __attribute__((packed)) {
    uint8_t report_id;  // = HID_REPORT_ID
    int16_t dx;
    int16_t dy;
    int8_t wheel;
    uint8_t buttons[2];
} hid_report_t;
```

## Integration with hid_hardware

The `hid_hardware` package automatically reads the generated URDF and creates the appropriate state interfaces. No manual configuration needed!

## Troubleshooting

### Build fails with "schema file not found"

Ensure your schema file path is relative to your package root or use an absolute path.

### Wrong number of state interfaces

Check that your schema fields match what your device actually sends. The generator creates one state interface per field (or per array element).

### Device not connecting

1. Check USB vendor/product IDs match your device
2. Verify udev rules for device permissions
3. Check `dmesg` for USB enumeration

## License

Apache-2.0
