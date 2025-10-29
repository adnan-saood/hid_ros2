# Basic Concepts

Understanding the core concepts of HID ROS 2 will help you make the most of the framework.

## Architecture Overview

HID ROS 2 follows a **schema-driven** architecture where everything originates from a single YAML file:

```
YAML Schema (schema/hid_device.yaml)
    │
    ├──> ROS 2 Files (auto-generated)
    │    ├── URDF/Xacro
    │    ├── Controller Configuration
    │    └── Launch Files
    │
    └──> Firmware Files (auto-generated)
         ├── HID Descriptor
         └── Report Structures
```

### Single Source of Truth

The YAML schema is the **only** file you need to maintain. All other files are automatically generated during the build process, ensuring consistency between firmware and ROS 2.

## Core Components

### 1. YAML Schema

Defines your device's interface:
- Device identification (VID/PID)
- Report IDs
- Data fields with types
- ROS 2 configuration

### 2. Code Generator (`hid_descriptor_generator`)

Reads the YAML schema and generates:
- **C Headers**: HID descriptors and structs for firmware
- **URDF**: Robot description with state/command interfaces
- **Configuration**: Controller parameters
- **Launch Files**: Ready-to-use launch files

### 3. Hardware Interface (`hid_hardware`)

A `ros2_control` hardware interface plugin that:
- Opens and manages USB HID device connection
- Reads input reports from device
- Writes output reports to device
- Exposes state/command interfaces to controllers

### 4. Controllers

ROS 2 controllers that use the state/command interfaces:
- **`hid_generic_broadcaster`**: Publishes sensor data
- **`hid_diagnostic_broadcaster`**: Monitors device health
- **`signal_command_controller`**: Example command controller

### 5. Developer Tools (`hid_tools`)

Command-line utilities for development:
- **`validate_schema`**: Schema validation
- **`inspect_device`**: Live USB monitoring
- **`generate_hid_descriptor`**: Standalone descriptor generation

## Data Flow

### Input Reports (Device → ROS 2)

1. **Firmware** sends HID input report over USB
2. **Hardware Interface** reads report via hidapi
3. **Hardware Interface** converts bytes to typed values
4. **Hardware Interface** updates state interfaces
5. **Controllers** read state interfaces
6. **Controllers** publish to ROS 2 topics

```
Device → USB → Hardware Interface → State Interfaces → Controllers → Topics
```

### Output Reports (ROS 2 → Device)

1. **Controllers** subscribe to ROS 2 topics
2. **Controllers** write to command interfaces
3. **Hardware Interface** reads command interfaces
4. **Hardware Interface** converts to bytes
5. **Hardware Interface** sends HID output report
6. **Firmware** receives and processes report

```
Topics → Controllers → Command Interfaces → Hardware Interface → USB → Device
```

## Type System

HID ROS 2 provides native support for multiple data types with automatic conversion.

### Supported Types

| Type      | Size    | Range                          | Use Case                |
|-----------|---------|--------------------------------|-------------------------|
| `uint8`   | 1 byte  | 0 to 255                       | Bytes, small values     |
| `int8`    | 1 byte  | -128 to 127                    | Small signed values     |
| `uint16`  | 2 bytes | 0 to 65,535                    | Medium unsigned values  |
| `int16`   | 2 bytes | -32,768 to 32,767              | Sensor readings         |
| `uint32`  | 4 bytes | 0 to 4,294,967,295             | Timestamps, counters    |
| `int32`   | 4 bytes | -2,147,483,648 to 2,147,483,647| Large signed values     |
| `float32` | 4 bytes | ±3.4e±38 (7 digits precision)  | Floating-point data     |
| `float64` | 8 bytes | ±1.7e±308 (15 digits)          | High-precision values   |

### Byte Order

All multi-byte values use **little-endian** byte order, which is standard for USB HID and most modern microcontrollers (ARM, x86).

### Type Conversion

The hardware interface automatically handles conversion:
- **Reading**: Bytes → Typed value → `double` (ros2_control interface)
- **Writing**: `double` → Typed value → Bytes

Example for `float32`:
```cpp
// Reading
uint8_t bytes[4] = {0x00, 0x00, 0x80, 0x3F};  // From USB
float value;
memcpy(&value, bytes, 4);                      // value = 1.0
double interface_value = static_cast<double>(value);

// Writing
double interface_value = 2.5;
float value = static_cast<float>(interface_value);
uint8_t bytes[4];
memcpy(bytes, &value, 4);                      // To USB
```

## State and Command Interfaces

### State Interfaces (Read-Only)

Represent sensor data coming **from** the device. Each field in your YAML schema creates a state interface.

Example schema:
```yaml
fields:
  - name: "temperature"
    type: "float32"
  - name: "pressure"
    type: "int16"
```

Creates state interfaces:
- `sensor_name/temperature`
- `sensor_name/pressure`

Controllers can read these values but not modify them.

### Command Interfaces (Write)

Represent commands going **to** the device. Created when you define output fields or use bidirectional mode.

Example schema with outputs:
```yaml
outputs:
  - name: "led_brightness"
    type: "uint8"
  - name: "motor_speed"
    type: "int16"
```

Creates command interfaces:
- `actuator_name/led_brightness`
- `actuator_name/motor_speed`

Controllers write to these interfaces, and values are sent to the device.

## Report IDs

HID supports multiple report types using report IDs (1-255).

### Input Reports

Data from device to computer (sensors, buttons, etc.):
```yaml
input_report_id: 2
```

### Output Reports

Data from computer to device (commands, configuration, etc.):
```yaml
output_report_id: 1
```

### Why Different IDs?

Having separate IDs allows:
- Different structures for input vs. output
- Unambiguous report identification
- Compliance with HID specification

## Hardware Interface Lifecycle

The hardware interface follows the `ros2_control` lifecycle:

1. **on_init**: Parse parameters, validate configuration
2. **on_configure**: Open USB device connection
3. **on_activate**: Start reading/writing reports
4. **read**: Read input report, update state interfaces
5. **write**: Read command interfaces, send output report
6. **on_deactivate**: Stop communication
7. **on_cleanup**: Close USB device

### Update Cycle

The `read()` and `write()` methods are called at the controller manager's update rate:

```
┌─────────────────────────────────────┐
│     Controller Manager Loop         │
│                                     │
│  1. hardware_interface.read()       │
│  2. controllers.update()            │
│  3. hardware_interface.write()      │
│                                     │
│  (Repeat at update_rate Hz)         │
└─────────────────────────────────────┘
```

## YAML Schema Structure

A complete schema has three main sections:

### 1. Device Identification
```yaml
device_name: "my_device"
vendor_id: "0x1234"
product_id: "0x5678"
```

### 2. ROS 2 Configuration
```yaml
sensor_name: "sensor"
frame_id: "sensor_frame"
update_rate: 250
```

### 3. Data Fields
```yaml
fields:
  - name: "field1"
    type: "float32"
    description: "Description"
```

## Generated Files

For each schema, the build process generates:

### For ROS 2
- **URDF** (`urdf/hid_robot.urdf.xacro`): Hardware description
- **Config** (`config/controllers.yaml`): Controller parameters
- **Launch** (`launch/hid.launch.py`): Launch file with spawners

### For Firmware
- **Header** (`include/package/hid_descriptor.h`): HID descriptor and structs

All files are generated in the build directory and installed to the install directory.

## Integration with ros2_control

HID ROS 2 is built on top of `ros2_control`, the standard control framework for ROS 2.

### Benefits

- **Standard Interface**: Works with any ros2_control controller
- **Tool Support**: Use `ros2 control` CLI commands
- **Ecosystem**: Compatible with existing controllers and tools
- **Architecture**: Clean separation of hardware and control logic

### ros2_control Components

1. **Hardware Interface**: `hid_hardware` (this package)
2. **Controller Manager**: Manages lifecycle and updates
3. **Controllers**: Read/write interfaces, publish topics
4. **Resource Manager**: Manages hardware resources

## Next Steps

- Learn about [Schema Reference](schema_reference.md) for detailed YAML syntax
- Understand [Code Generation](code_generation.md) process
- Explore [Hardware Interface](hardware_interface.md) implementation
- Read about [Controllers](controllers.md) available
