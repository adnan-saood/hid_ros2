# HID Tools

Developer tools for HID device development and debugging with ROS 2.

## Overview

The `hid_tools` package provides three essential command-line tools to streamline HID device development:

1. **`validate_schema`** - Validate YAML schema files before code generation
2. **`inspect_device`** - Live USB HID device inspector and monitor
3. **`generate_hid_report_device`** - Generate C code HID descriptors for MCU firmware

These tools help developers create, debug, and validate HID devices without manually writing error-prone code.

## Installation

### Build the Package

```bash
cd ~/dev/hid_ros2
colcon build --packages-select hid_tools
source install/setup.bash
```

### Python Dependencies

The `inspect_device` tool requires the `hidapi` Python library:

```bash
pip install hidapi
```

## Tools

### 1. validate_schema

Validates HID device YAML schemas for correctness and best practices.

**Features:**
- Checks for required fields (`device_name`, `vendor_id`, `product_id`, `sensor_name`)
- Validates field types (`uint8`, `int16`, `float32`, etc.)
- Verifies report IDs are in valid range (1-255)
- Detects duplicate field names
- Warns about deprecated formats and common issues
- Strict mode treats warnings as errors

**Usage:**

```bash
# Validate a single schema
ros2 run hid_tools validate_schema my_device.yaml

# Validate multiple schemas
ros2 run hid_tools validate_schema schema1.yaml schema2.yaml

# Strict mode (warnings become errors)
ros2 run hid_tools validate_schema --strict my_device.yaml
```

**Example Output:**

```
Validating: signal_generator.yaml
======================================================================

✅ Schema is valid!
```

or with issues:

```
Validating: bad_schema.yaml
======================================================================

❌ ERRORS (2):
  • Missing required field: 'device_name'
  • inputs.velocity: invalid type 'uint128'. Valid types: uint8, int8, uint16, ...

⚠️  WARNINGS (1):
  • No 'input_report_id' specified, will default to 1

❌ Schema has 2 error(s)
```

### 2. inspect_device

Live USB HID device inspector and report monitor.

**Features:**
- List all connected HID devices
- Open device by VID/PID
- Monitor input reports in real-time
- Send output reports to devices
- Display raw bytes and decoded values
- Non-blocking read with rate statistics
- Auto-decode as uint32, float32, float32 arrays

**Usage:**

```bash
# List all HID devices
ros2 run hid_tools inspect_device --list

# Monitor input reports from a device
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000

# Read exactly 10 reports
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 --count 10

# Read for 5 seconds
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 --duration 5.0

# Send output report (report ID 1, three float32 values: 1.0, 2.0, 3.0)
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 \
  --send 1:0x00,0x00,0x80,0x3F,0x00,0x00,0x00,0x40,0x00,0x00,0x40,0x40
```

**Example Output:**

```
📊 Reading input reports (Ctrl+C to stop)...
====================================================================================================

[0001] t=0.012s  Report ID: 2
  Raw bytes (9): 02 00 00 00 00 CD CC 4C 3F
  Decoded:
    uint32[0]: 0
    float32[0]: 0.000000
    float32[0..1]: ['0.000000', '0.800000']

[0002] t=0.112s  Report ID: 2
  Raw bytes (9): 02 9A 99 19 3F 33 33 D3 3F
  Decoded:
    uint32[0]: 1050673562
    float32[0]: 0.600000
    float32[0..1]: ['0.600000', '1.650000']

⏸️  Stopped by user

📈 Statistics:
   Reports received: 25
   Duration: 2.51s
   Average rate: 9.96 reports/sec
```

### 3. generate_hid_report_device

Generates C code HID descriptor from YAML schema for MCU firmware.

**Features:**
- Auto-generates complete HID report descriptor
- Creates C header with descriptor array and structs
- Calculates report sizes automatically
- Generates type-safe report structures
- Supports both input and output reports
- Handles multi-field reports with various types

**Usage:**

```bash
# Print descriptor summary
ros2 run hid_tools generate_hid_report_device my_device.yaml

# Generate C header file
ros2 run hid_tools generate_hid_report_device my_device.yaml -o hid_descriptor.h

# Print full descriptor to stdout
ros2 run hid_tools generate_hid_report_device my_device.yaml --print

# Just the summary
ros2 run hid_tools generate_hid_report_device my_device.yaml --summary
```

**Example Output (Summary):**

```
📋 HID Descriptor Summary
======================================================================
Device: Signal Generator
Descriptor size: 43 bytes

Input Report:
  Report ID: 2
  Size: 8 bytes
  Fields: 2
    - timestamp: float32 (4 bytes)
    - result: float32 (4 bytes)

Output Report:
  Report ID: 1
  Size: 12 bytes
  Fields: 3
    - amplitude: float32 (4 bytes)
    - frequency: float32 (4 bytes)
    - phase: float32 (4 bytes)
```

**Generated Header Example:**

```c
/*
 * HID Descriptor for Signal Generator
 * Generated from: signal_generator.yaml
 *
 * Auto-generated by hid_tools - DO NOT EDIT MANUALLY
 */

#ifndef SIGNAL_GENERATOR_HID_DESCRIPTOR_H
#define SIGNAL_GENERATOR_HID_DESCRIPTOR_H

#include <stdint.h>

// Report IDs
#define HID_INPUT_REPORT_ID   2
#define HID_OUTPUT_REPORT_ID  1

// Report sizes (in bytes, excluding report ID)
#define HID_INPUT_REPORT_SIZE  8
#define HID_OUTPUT_REPORT_SIZE 12

// HID Report Descriptor
static const uint8_t hid_report_descriptor[] = {
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x02, 0x15, 0x00, 0x26,
    0xFF, 0x00, 0x75, 0x08, 0x95, 0x08, 0x81, 0x02, 0x85, 0x01, 0x15, 0x00,
    0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x91, 0x02, 0xC0
};

#define HID_REPORT_DESCRIPTOR_SIZE (sizeof(hid_report_descriptor))

// Input Report Structure
typedef struct __attribute__((packed)) {
    uint8_t report_id;  // Must be HID_INPUT_REPORT_ID
    float timestamp;
    float result;
} HIDInputReport;

// Output Report Structure
typedef struct __attribute__((packed)) {
    uint8_t report_id;  // Must be HID_OUTPUT_REPORT_ID
    float amplitude;
    float frequency;
    float phase;
} HIDOutputReport;

#endif // SIGNAL_GENERATOR_HID_DESCRIPTOR_H
```

## Workflow Examples

### Schema Development Workflow

```bash
# 1. Create YAML schema
vim my_device.yaml

# 2. Validate schema
ros2 run hid_tools validate_schema my_device.yaml

# 3. Generate HID descriptor for firmware
ros2 run hid_tools generate_hid_report_device my_device.yaml -o firmware/hid_desc.h

# 4. Generate ROS 2 files
ros2 run hid_descriptor_generator generate_hid_files my_device.yaml

# 5. Build and test
colcon build
```

### Device Debugging Workflow

```bash
# 1. List devices to find VID/PID
ros2 run hid_tools inspect_device --list

# 2. Monitor input reports
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000

# 3. Test output reports
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 \
  --send 1:0x00,0x00,0x80,0x3F

# 4. Compare with ROS 2 controller
ros2 topic echo /hid_broadcaster/sensor_states
```

## Supported Data Types

All tools support the following data types:

- **Unsigned integers**: `uint8`, `uint16`, `uint32`, `uint64`
- **Signed integers**: `int8`, `int16`, `int32`, `int64`
- **Floating point**: `float32`, `float64`

## Error Handling

### validate_schema

- **Errors**: Critical issues that prevent code generation (missing fields, invalid types)
- **Warnings**: Non-critical issues that may cause problems (missing report IDs, deprecated formats)
- **Exit codes**: 0 for success, 1 for errors (or warnings in strict mode)

### inspect_device

- **Device not found**: Lists available devices and exits
- **Permission denied**: Check udev rules for USB HID access
- **No data**: Reports statistics even if zero reports received

### generate_hid_report_device

- **YAML errors**: Reports parsing errors with line numbers
- **Unknown types**: Uses uint8 as fallback with warning
- **Empty reports**: Generates valid descriptor with zero-length reports

## Tips

1. **Always validate** schemas before generating code to catch errors early
2. **Use inspect_device** to verify MCU firmware is sending correct report IDs and data
3. **Compare raw bytes** from inspect_device with ROS 2 topic values to debug conversion issues
4. **Generate descriptors** for both PC and MCU to ensure compatibility
5. **Use --strict mode** in CI/CD pipelines to enforce schema quality

## See Also

- [hid_descriptor_generator](../hid_descriptor_generator/README.md) - Main code generator
- [QUICKSTART.md](../QUICKSTART.md) - Getting started guide
- [DYNAMIC_INTERFACES.md](../DYNAMIC_INTERFACES.md) - Type system documentation

## License

Apache License 2.0
