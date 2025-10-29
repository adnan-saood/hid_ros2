# Developer Tools

Command-line tools for HID device development.

## Overview

The `hid_tools` package provides three essential tools:

1. **validate_schema**: YAML schema validation
2. **inspect_device**: Live USB HID device monitor
3. **generate_hid_descriptor**: HID descriptor generator

## validate_schema

Validates YAML schemas before code generation.

### Usage

```bash
# Validate a schema
ros2 run hid_tools validate_schema schema/hid_device.yaml

# Strict mode (warnings as errors)
ros2 run hid_tools validate_schema --strict schema/hid_device.yaml
```

### Features

- Required field checking
- Type validation
- Report ID validation
- Duplicate field detection
- Best practice warnings

## inspect_device

Live USB HID device inspector.

### Usage

```bash
# List all HID devices
ros2 run hid_tools inspect_device --list

# Monitor a specific device
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000

# Read 10 reports
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 --count 10

# Read for 5 seconds
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 --duration 5.0

# Send output report
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000 \
  --send 1:0x00,0x00,0x80,0x3F
```

### Features

- Device enumeration
- Live report monitoring
- Output report sending
- Multiple data decodings
- Rate statistics

## generate_hid_descriptor

Generates HID descriptor C code from schema.

### Usage

```bash
# Print summary
ros2 run hid_tools generate_hid_descriptor schema/hid_device.yaml

# Generate header file
ros2 run hid_tools generate_hid_descriptor schema/hid_device.yaml -o hid_desc.h

# Print full descriptor
ros2 run hid_tools generate_hid_descriptor schema/hid_device.yaml --print
```

### Features

- Complete HID descriptor generation
- Type-safe report structures
- Firmware-ready output
- Standalone operation

For more examples, see the [examples section](examples/signal_generator.md).
