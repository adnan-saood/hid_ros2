# Code Generation

How HID ROS 2 generates code from YAML schemas.

## Overview

The code generation process transforms your YAML schema into complete ROS 2 integration and firmware support files. This happens **automatically during the build process** via a CMake function.

## How It Works

### Build-Time Generation

Code generation is integrated into the CMake build system:

1. You add `hid_generate()` to your `CMakeLists.txt`
2. When you run `colcon build`, CMake invokes the generator
3. Python script reads your YAML schema
4. All necessary files are generated
5. Files are installed to the install directory

### The CMake Function

In your package's `CMakeLists.txt`:

```cmake
# Find the generator package
find_package(hid_descriptor_generator REQUIRED)

# Include the CMake function
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

# Call the generator
hid_generate(
  SCHEMA_FILE "schema/hid_device.yaml"
)
```

**What This Does:**
- Creates custom build target
- Sets up dependencies on schema file
- Calls Python generator script when schema changes
- Installs generated files to correct locations
- Exports include directories for your package

### Regeneration

Files are regenerated automatically when:
- Schema file is modified
- Clean build is performed
- Generator script is updated

**Incremental Builds:**
CMake tracks the schema file as a dependency, so changes trigger regeneration without full rebuild.

## Generated Files

### ROS 2 Side

**URDF (urdf/hid_robot.urdf.xacro)**
- Hardware interface definition using `hid_hardware` plugin
- State interface declarations for each input field
- Command interface declarations for each output field (if defined)
- Robot description with proper parameters

**Controller Config (config/controllers.yaml)**
- Controller manager configuration
- `hid_generic_broadcaster` setup
- Update rates and publishing rates
- Interface mappings

**Launch File (launch/hid.launch.py)**
- Loads URDF into robot_state_publisher
- Spawns hardware interface via controller_manager
- Spawns controllers
- Sets up TF static broadcaster (world → frame_id)

### Firmware Side

**HID Descriptor Header (include/<package>/hid_descriptor.h)**
- USB HID report descriptor array (`hid_report_descriptor[]`)
- Descriptor size constant (`HID_REPORT_DESCRIPTOR_SIZE`)
- Input report structure with `__attribute__((packed))`
- Output report structure (if outputs defined)
- Report ID constants
- Report size constants
- Field documentation in comments

## Generation Process Details

### Step 1: Schema Parsing

```
YAML Schema → Python Parser → Internal Data Structure
```

The generator:
- Validates YAML syntax
- Checks required fields
- Validates types
- Resolves defaults

### Step 2: HID Descriptor Generation

```
Field Definitions → HID Item Encoding → Descriptor Bytes
```

For each field:
- Determines HID item type (Input/Output)
- Calculates bit/byte sizes
- Encodes as HID descriptor items
- Adds report ID items

### Step 3: URDF Generation

```
Fields + Configuration → URDF Template → Robot Description
```

Creates:
- Hardware component with `hid_hardware` plugin
- Parameters (VID, PID, report IDs)
- State interfaces for each input field
- Command interfaces for each output field

### Step 4: Controller Configuration

```
Schema Metadata → YAML Template → Controller Config
```

Configures:
- `hid_generic_broadcaster` with all state interfaces
- Publishing rate
- Controller manager parameters

### Step 5: Launch File Generation

```
Package Info + Config → Python Template → Launch File
```

Generates launch file that:
- Loads URDF
- Starts controller_manager
- Spawns controllers in correct order
- Sets up TF tree

## File Locations

### Build Directory

During build, files are generated to:
```
build/<package_name>/generated/
  ├── include/<package_name>/hid_descriptor.h
  ├── urdf/hid_robot.urdf.xacro
  ├── config/controllers.yaml
  └── launch/hid.launch.py
```

### Install Directory

After build, files are installed to:
```
install/<package_name>/
  ├── include/<package_name>/hid_descriptor.h
  ├── share/<package_name>/
  │   ├── urdf/hid_robot.urdf.xacro
  │   ├── config/controllers.yaml
  │   ├── launch/hid.launch.py
  │   └── schema/hid_device.yaml
```

## Type Mapping

### YAML to HID Descriptor

| YAML Type | HID Report Size | Bits | Bytes |
|-----------|----------------|------|-------|
| `uint8`   | 8              | 8    | 1     |
| `int8`    | 8              | 8    | 1     |
| `uint16`  | 16             | 16   | 2     |
| `int16`   | 16             | 16   | 2     |
| `uint32`  | 32             | 32   | 4     |
| `int32`   | 32             | 32   | 4     |
| `float32` | 32             | 32   | 4     |
| `float64` | 64             | 64   | 8     |

### YAML to C Types

| YAML Type | C Type (Firmware) | ROS 2 Interface |
|-----------|------------------|-----------------|
| `uint8`   | `uint8_t`        | `double`        |
| `int8`    | `int8_t`         | `double`        |
| `uint16`  | `uint16_t`       | `double`        |
| `int16`   | `int16_t`        | `double`        |
| `uint32`  | `uint32_t`       | `double`        |
| `int32`   | `int32_t`        | `double`        |
| `float32` | `float`          | `double`        |
| `float64` | `double`         | `double`        |

**Note:** All ros2_control interfaces use `double` regardless of source type. Type conversion happens in the hardware interface.

## Advanced Usage

### Multiple Schemas

For packages with multiple devices:

```cmake
hid_generate(SCHEMA_FILE "schema/device1.yaml")
hid_generate(SCHEMA_FILE "schema/device2.yaml")
```

Each generates to the same output directories - files are overwritten. For multiple devices, use separate packages.

### Custom Output Directories

The generator uses fixed output paths relative to the build directory. To customize, create wrapper scripts.

### Manual Invocation

The Python script can be called manually (advanced):

```bash
python3 /path/to/generate_hid_files.py \
  --schema schema/device.yaml \
  --output-dir output/ \
  --package-name my_package
```

But this is not recommended - use the CMake function instead.

## Customization

### Modifying Generated Files

**Not Recommended:**
Generated files are overwritten on every build.

**Recommended Approaches:**

1. **Modify the schema** - Change YAML and regenerate
2. **Wrapper files** - Import generated URDF/launch and extend
3. **Custom controllers** - Create controllers that use the generated interfaces

### Example: Custom Launch File

Instead of modifying generated `hid.launch.py`, create your own:

```python
# my_custom.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include generated launch file
    hid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('my_package'),
                        'launch', 'hid.launch.py')
        ])
    )

    # Add your custom nodes
    # ...

    return LaunchDescription([hid_launch, ...])
```

## Debugging Generation

### Verbose Output

```bash
colcon build --packages-select my_package --event-handlers console_direct+
```

Look for: `Generating HID descriptor and ROS2 files from ...`

### Check Generated Files

```bash
# View what was generated
find build/my_package/generated -type f

# View build directory structure
tree build/my_package/generated
```

### Validate Schema First

```bash
ros2 run hid_tools validate_schema schema/device.yaml --strict
```

Catches errors before build.

## Troubleshooting

### Generation Not Running

- Check schema file path in CMakeLists.txt
- Verify `hid_descriptor_generator` is found
- Look for CMake errors in build output

### Files Not Installed

- Check `install()` commands in CMakeLists.txt
- Verify build succeeded without errors
- Check `install/` directory structure

### Wrong Content

- Validate schema with `validate_schema`
- Check for YAML syntax errors
- Verify field types are valid

## See Also

- [Schema Reference](schema_reference.md) - Complete YAML reference
- [Complete Tutorial](tutorial_custom_device.md) - Step-by-step walkthrough
- [API Reference](api_reference.md) - CMake function reference
