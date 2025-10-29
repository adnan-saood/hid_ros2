# API Reference

API documentation for HID ROS 2 components.

## CMake API

### hid_generate()

The primary interface for code generation from YAML schemas.

**Usage in CMakeLists.txt:**

```cmake
find_package(hid_descriptor_generator REQUIRED)
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

hid_generate(
  SCHEMA_FILE "schema/hid_device.yaml"
)
```

**Parameters:**
- `SCHEMA_FILE` (required): Path to YAML schema file (relative to CMakeLists.txt or absolute)

**Generated Files:**
- `include/${PROJECT_NAME}/hid_descriptor.h` - HID descriptor C header
- `urdf/hid_robot.urdf.xacro` - URDF with state/command interfaces
- `config/controllers.yaml` - Controller configuration
- `launch/hid.launch.py` - Launch file

**Build Integration:**
- Automatically runs during `colcon build`
- Regenerates files when schema changes
- Exports include directories for downstream packages

## Python API

The Python generator script is called internally by the CMake function. Direct Python usage is not recommended for normal use cases.

**Internal Script Location:**
```
lib/hid_descriptor_generator/generate_hid_files.py
```

## C++ API

### hid_hardware::HidHardware

Hardware interface implementation.

**Class**: `hid_hardware::HidHardware`

**Inherits**: `hardware_interface::SystemInterface`

**Methods**:
- `on_init()`: Initialize hardware interface
- `on_configure()`: Open HID device
- `on_activate()`: Start communication
- `read()`: Read input report, update states
- `write()`: Read commands, send output report
- `on_deactivate()`: Stop communication
- `on_cleanup()`: Close device

## ROS 2 Interfaces

### State Interfaces

Format: `<sensor_name>/<field_name>`

Example:
- `hid_sensor/position_x`
- `hid_sensor/velocity`

### Command Interfaces

Format: `<actuator_name>/<output_name>`

Example:
- `hid_actuator/led_brightness`
- `hid_actuator/motor_speed`

## Command-Line Tools

### validate_schema

```bash
ros2 run hid_tools validate_schema [OPTIONS] SCHEMA_FILE...

Options:
  --strict    Treat warnings as errors
  --help      Show help message
```

### inspect_device

```bash
ros2 run hid_tools inspect_device [OPTIONS]

Options:
  --list              List all HID devices
  --vid VID           Vendor ID (hex)
  --pid PID           Product ID (hex)
  --count N           Read N reports then stop
  --duration SECONDS  Read for SECONDS then stop
  --send REPORT       Send output report
```

### generate_hid_descriptor

```bash
ros2 run hid_tools generate_hid_descriptor [OPTIONS] SCHEMA_FILE

Options:
  -o, --output FILE   Write to file
  --print            Print descriptor
  --summary          Print summary only
```

For detailed usage, see [Developer Tools](developer_tools.md).
