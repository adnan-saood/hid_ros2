# Schema Reference

Complete reference for the YAML schema format used by HID ROS 2.

## Overview

The YAML schema is the single source of truth for your HID device integration. This document describes all available fields and their usage.

## Required Fields

### Device Information

#### `device_name`
- **Type**: String
- **Required**: Yes
- **Description**: Unique identifier for your device
- **Example**: `"my_sensor_device"`
- **Constraints**: No spaces, use underscores

#### `vendor_id`
- **Type**: String (hexadecimal)
- **Required**: Yes
- **Description**: USB Vendor ID
- **Format**: `"0xVVVV"` where VVVV is 4 hex digits
- **Example**: `"0x046d"` (Logitech)

#### `product_id`
- **Type**: String (hexadecimal)
- **Required**: Yes
- **Description**: USB Product ID
- **Format**: `"0xPPPP"` where PPPP is 4 hex digits
- **Example**: `"0xc07e"`

### ROS 2 Configuration

#### `sensor_name`
- **Type**: String
- **Required**: Yes
- **Description**: Name used for state/command interfaces
- **Example**: `"hid_sensor"`
- **Note**: Appears in interface names like `sensor_name/field_name`

#### `frame_id`
- **Type**: String
- **Required**: Yes
- **Description**: TF frame ID for this device
- **Example**: `"sensor_frame"`
- **Note**: Used in TF broadcasts and sensor messages

#### `update_rate`
- **Type**: Integer
- **Required**: Yes
- **Description**: Update rate in Hz
- **Range**: 1-1000
- **Example**: `250`
- **Recommendation**:
  - Gaming devices: 500-1000 Hz
  - Sensors: 100-250 Hz
  - Slow sensors: 10-50 Hz

## Optional Fields

### Report IDs

#### `input_report_id`
- **Type**: Integer
- **Required**: No
- **Default**: 1
- **Description**: Report ID for input reports (device → computer)
- **Range**: 1-255
- **Example**: `2`

#### `output_report_id`
- **Type**: Integer
- **Required**: No (only if outputs defined)
- **Description**: Report ID for output reports (computer → device)
- **Range**: 1-255
- **Example**: `1`

## Field Definitions

### Input Fields

The `fields` array defines the structure of input reports (data from device).

```yaml
fields:
  - name: "field_name"
    type: "type_name"
    description: "Optional description"
    count: 1  # Optional, for arrays
```

#### Field Properties

##### `name`
- **Type**: String
- **Required**: Yes
- **Description**: Field identifier
- **Constraints**:
  - No spaces
  - Must be unique within schema
  - Use lowercase with underscores
- **Example**: `"position_x"`, `"button_state"`

##### `type`
- **Type**: String
- **Required**: Yes
- **Description**: Data type
- **Valid Values**:
  - `uint8`, `uint16`, `uint32`, `uint64`
  - `int8`, `int16`, `int32`, `int64`
  - `float32`, `float64`
- **Example**: `"float32"`

##### `description`
- **Type**: String
- **Required**: No
- **Description**: Human-readable description
- **Usage**: Appears in generated code comments
- **Example**: `"X position in meters"`

##### `count`
- **Type**: Integer
- **Required**: No
- **Default**: 1
- **Description**: Array size for multi-value fields
- **Range**: 1-256
- **Example**: `3` for `[x, y, z]`
- **Note**: Creates separate interfaces: `name_0`, `name_1`, etc.

### Output Fields

The `outputs` array defines output report structure (commands to device).

```yaml
outputs:
  - name: "command_name"
    type: "type_name"
    description: "Optional description"
    count: 1  # Optional
```

Properties are identical to input fields.

## Complete Example

```yaml
# Device identification
device_name: "imu_sensor"
vendor_id: "0xCAFE"
product_id: "0x4000"

# Report IDs
input_report_id: 2
output_report_id: 1

# ROS 2 configuration
sensor_name: "imu"
frame_id: "imu_link"
update_rate: 200

# Input data structure
fields:
  # Timestamp
  - name: "timestamp"
    type: "uint32"
    description: "Millisecond timestamp"

  # Acceleration (3-axis)
  - name: "accel"
    type: "int16"
    count: 3
    description: "Acceleration in mg (milli-g)"

  # Gyroscope (3-axis)
  - name: "gyro"
    type: "int16"
    count: 3
    description: "Angular velocity in mdps"

  # Magnetometer (3-axis)
  - name: "mag"
    type: "int16"
    count: 3
    description: "Magnetic field in mGauss"

  # Temperature
  - name: "temperature"
    type: "int16"
    description: "Temperature in 0.01°C"

  # Status byte
  - name: "status"
    type: "uint8"
    description: "Status flags bitfield"

# Output commands (optional)
outputs:
  # Sample rate configuration
  - name: "sample_rate"
    type: "uint16"
    description: "Desired sample rate in Hz"

  # Power mode
  - name: "power_mode"
    type: "uint8"
    description: "0=low power, 1=normal, 2=high performance"

  # Calibration trigger
  - name: "calibrate"
    type: "uint8"
    description: "Write 1 to start calibration"
```

## Type Reference

### Integer Types

| Type     | Bytes | Signed | Min Value        | Max Value       |
|----------|-------|--------|------------------|-----------------|
| `uint8`  | 1     | No     | 0                | 255             |
| `int8`   | 1     | Yes    | -128             | 127             |
| `uint16` | 2     | No     | 0                | 65,535          |
| `int16`  | 2     | Yes    | -32,768          | 32,767          |
| `uint32` | 4     | No     | 0                | 4,294,967,295   |
| `int32`  | 4     | Yes    | -2,147,483,648   | 2,147,483,647   |
| `uint64` | 8     | No     | 0                | 2^64 - 1        |
| `int64`  | 8     | Yes    | -2^63            | 2^63 - 1        |

### Floating-Point Types

| Type      | Bytes | Precision | Range (approx) |
|-----------|-------|-----------|----------------|
| `float32` | 4     | 7 digits  | ±3.4e±38       |
| `float64` | 8     | 15 digits | ±1.7e±308      |

### Type Selection Guidelines

**Use `uint8`/`uint16` when:**
- Value is always positive
- Representing counts, IDs, or flags
- Memory efficiency is important

**Use `int8`/`int16` when:**
- Value can be negative
- Representing sensor data (temperature, acceleration, etc.)

**Use `uint32`/`uint64` when:**
- Representing timestamps
- Large counts or IDs
- Checksums or hashes

**Use `float32` when:**
- Need decimal precision
- Representing physical measurements
- Most sensor applications

**Use `float64` when:**
- Need high precision
- Accumulating values
- Scientific calculations

## Array Fields

Arrays create multiple state/command interfaces with indexed names.

### Single Value
```yaml
- name: "value"
  type: "float32"
```
Creates: `sensor_name/value`

### Array Value
```yaml
- name: "position"
  type: "float32"
  count: 3
```
Creates:
- `sensor_name/position_0`
- `sensor_name/position_1`
- `sensor_name/position_2`

## Validation

Use the schema validator to check your YAML:

```bash
ros2 run hid_tools validate_schema your_schema.yaml --strict
```

### Common Errors

**Missing required field:**
```
❌ Missing required field: 'device_name'
```

**Invalid type:**
```
❌ fields.velocity: invalid type 'float'. Valid types: uint8, int8, ...
```

**Invalid vendor_id format:**
```
❌ vendor_id must be in format '0xVVVV' (e.g., '0x046d')
```

**Duplicate field name:**
```
❌ Duplicate field name: 'position'
```

## Best Practices

### Naming Conventions

- **device_name**: `lowercase_with_underscores`
- **sensor_name**: `short_descriptive_name`
- **frame_id**: Follow TF naming (usually `device_name_link`)
- **field names**: `descriptive_lowercase`

### Field Ordering

Organize fields logically:
1. Timestamps first
2. Primary data (position, velocity, etc.)
3. Secondary data (status, flags, etc.)

### Type Selection

Choose the smallest type that fits your data:
- Saves USB bandwidth
- Reduces processing overhead
- Improves performance

### Documentation

Always add descriptions:
- Helps future developers
- Appears in generated code
- Documents units and ranges

### Report IDs

Standard convention:
- Input reports: ID 2 (or higher)
- Output reports: ID 1

This separates input/output clearly.

## See Also

- [Code Generation](code_generation.md) - How schemas become code
- [Type System](advanced/type_system.md) - Deep dive into type handling
- [Examples](examples/dummy_pose.md) - Complete example schemas
