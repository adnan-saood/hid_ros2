# Tutorial: Create Your Custom HID Device Integration

This tutorial walks you through creating a complete custom HID device integration from scratch, from cloning the repository to running your device with ROS 2.

## What You'll Build

A custom 3-axis accelerometer HID device integration with:
- YAML schema defining the device
- Automatic code generation
- Generic broadcaster publishing sensor data
- Complete ROS 2 launch system

**Time Required:** ~30 minutes

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- Basic familiarity with terminal commands
- A HID device (or you can test without hardware first)

## Step 1: Clone and Build the Base Repository

### 1.1 Create Workspace and Clone

```bash
# Create a ROS 2 workspace
mkdir -p ~/hid_ws/src
cd ~/hid_ws/src

# Clone the HID ROS 2 repository
git clone https://github.com/adnan-saood/hid_ros2.git

# Return to workspace root
cd ~/hid_ws
```

### 1.2 Install Dependencies

```bash
# Update rosdep
rosdep update

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 1.3 Build the Base Packages

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the core packages first
colcon build --packages-select hid_descriptor_generator hid_hardware

# Source the workspace
source install/setup.bash
```

**Verify Installation:**
```bash
# Check that packages are built
ros2 pkg list | grep hid

# You should see:
# hid_descriptor_generator
# hid_hardware
# hid_diagnostic_broadcaster
# hid_generic_broadcaster
# (and others)
```

## Step 2: Create Your Custom Device Package

### 2.1 Create Package Structure

```bash
cd ~/hid_ws/src

# Create a new ROS 2 package
ros2 pkg create --build-type ament_cmake my_accel_bringup \
  --dependencies rclcpp hardware_interface controller_manager

# Create directory structure
cd my_accel_bringup
mkdir -p schema
```

### 2.2 Configure CMakeLists.txt

Edit `CMakeLists.txt` to add HID generation:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_accel_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hid_descriptor_generator REQUIRED)

# Include HID generation function
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

# Generate HID files from schema
hid_generate(
  SCHEMA_FILE "schema/accelerometer.yaml"
)

# Install schema directory
install(DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 2.3 Update package.xml

Edit `package.xml` to add dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_accel_bringup</name>
  <version>0.1.0</version>
  <description>Custom accelerometer HID device integration</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>hardware_interface</depend>
  <depend>controller_manager</depend>

  <!-- HID dependencies -->
  <build_depend>hid_descriptor_generator</build_depend>
  <exec_depend>hid_hardware</exec_depend>
  <exec_depend>hid_generic_broadcaster</exec_depend>
  <exec_depend>controller_manager</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Step 3: Define Your Device Schema

### 3.1 Create Schema File

Create `schema/accelerometer.yaml`:

```yaml
# Device identification
device_name: "my_accelerometer"
vendor_id: "0xCAFE"      # Replace with your device's VID
product_id: "0x4001"     # Replace with your device's PID

# Report IDs
input_report_id: 2       # Device sends data with this ID
# output_report_id: 1    # Uncomment if device accepts commands

# ROS 2 configuration
sensor_name: "accel"
frame_id: "accel_link"
update_rate: 100         # Hz - adjust to your device

# Data fields
# These define the HID report structure
fields:
  # Timestamp from device
  - name: "timestamp"
    type: "uint32"
    description: "Millisecond timestamp from device"

  # 3-axis acceleration in milli-g (1000 = 1g)
  - name: "accel_x"
    type: "int16"
    description: "X-axis acceleration in milli-g"

  - name: "accel_y"
    type: "int16"
    description: "Y-axis acceleration in milli-g"

  - name: "accel_z"
    type: "int16"
    description: "Z-axis acceleration in milli-g"

  # Temperature in 0.01°C
  - name: "temperature"
    type: "int16"
    description: "Temperature in 0.01 degrees Celsius"

  # Status byte
  - name: "status"
    type: "uint8"
    description: "Device status flags"
```

### 3.2 Understanding the Schema

**Key Fields Explained:**

- **vendor_id/product_id**: Your USB device IDs (find with `lsusb`)
- **input_report_id**: ID byte that prefixes input reports from device
- **sensor_name**: Used in ROS 2 interface names (e.g., `accel/accel_x`)
- **frame_id**: TF frame for this sensor
- **update_rate**: How fast the controller manager reads from device
- **fields**: Define your HID report structure

**Type Selection:**
- `uint8/16/32`: Positive values only (0 to max)
- `int8/16/32`: Can be negative (e.g., acceleration can be negative)
- `float32/64`: Decimal values (uses more bytes)

## Step 4: Build Your Package

### 4.1 Validate Schema (Optional but Recommended)

```bash
# First, build hid_tools if not already built
cd ~/hid_ws
colcon build --packages-select hid_tools
source install/setup.bash

# Validate your schema
ros2 run hid_tools validate_schema \
  src/my_accel_bringup/schema/accelerometer.yaml --strict
```

You should see:
```
Validating: accelerometer.yaml
======================================================================

✅ Schema is valid!
```

### 4.2 Build Package

```bash
cd ~/hid_ws

# Build your package
colcon build --packages-select my_accel_bringup

# Check for errors in build output
# You should see: "Generating HID descriptor and ROS2 files..."

# Source the workspace
source install/setup.bash
```

### 4.3 Verify Generated Files

```bash
# Check generated files
find install/my_accel_bringup -type f

# You should see:
# install/my_accel_bringup/
#   include/my_accel_bringup/hid_descriptor.h
#   share/my_accel_bringup/
#     urdf/hid_robot.urdf.xacro
#     config/controllers.yaml
#     launch/hid.launch.py
#     schema/accelerometer.yaml
```

### 4.4 Inspect Generated Files

**View the HID descriptor header:**
```bash
cat install/my_accel_bringup/include/my_accel_bringup/hid_descriptor.h
```

This header is for your **firmware** - it contains:
- HID report descriptor array
- Input report structure (C struct)
- Report ID constants

**View the URDF:**
```bash
cat install/my_accel_bringup/share/my_accel_bringup/urdf/hid_robot.urdf.xacro
```

This defines the hardware interface with state interfaces for each field.

**View controller config:**
```bash
cat install/my_accel_bringup/share/my_accel_bringup/config/controllers.yaml
```

## Step 5: Set Up USB Permissions

### 5.1 Find Your Device

```bash
# List USB devices
lsusb

# Look for your device, e.g.:
# Bus 001 Device 005: ID cafe:4001 My Custom Device
```

### 5.2 Create udev Rule

```bash
# Create udev rule (replace CAFE and 4001 with your VID/PID)
sudo tee /etc/udev/rules.d/99-my-accelerometer.rules <<EOF
# My Accelerometer HID Device
SUBSYSTEM=="usb", ATTRS{idVendor}=="cafe", ATTRS{idProduct}=="4001", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="cafe", ATTRS{idProduct}=="4001", MODE="0666", GROUP="plugdev"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add yourself to plugdev group (if not already)
sudo usermod -a -G plugdev $USER

# Log out and back in for group changes to take effect
```

## Step 6: Test Without Hardware (Optional)

You can test the integration even without physical hardware:

### 6.1 Inspect HID Devices

```bash
# List all HID devices
ros2 run hid_tools inspect_device --list
```

If your device is connected, you'll see it listed.

## Step 7: Launch Your Device

### 7.1 Launch with Generic Broadcaster

```bash
cd ~/hid_ws
source install/setup.bash

# Launch your device integration
ros2 launch my_accel_bringup hid.launch.py
```

**Expected Output:**
```
[INFO] [controller_manager]: Loading controller 'hid_generic_broadcaster'
[INFO] [hid_hardware]: Successfully opened HID device cafe:4001
[INFO] [hid_hardware]: Device connected and ready
[INFO] [controller_manager]: Configured and activated 'hid_generic_broadcaster'
```

### 7.2 Monitor Data

Open a new terminal:

```bash
source ~/hid_ws/install/setup.bash

# List available topics
ros2 topic list

# You should see:
# /accel_broadcaster/accel_states

# Monitor the data
ros2 topic echo /accel_broadcaster/accel_states
```

### 7.3 Check State Interfaces

```bash
# List hardware interfaces
ros2 control list_hardware_interfaces

# You should see:
# state interfaces
#   accel/timestamp [available] [claimed]
#   accel/accel_x [available] [claimed]
#   accel/accel_y [available] [claimed]
#   accel/accel_z [available] [claimed]
#   accel/temperature [available] [claimed]
#   accel/status [available] [claimed]
```

## Step 8: Debug and Verify

### 8.1 Monitor Raw USB Data

```bash
# Watch raw HID reports from device
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4001

# You should see:
# [0001] t=0.010s  Report ID: 2
#   Raw bytes (14): 02 00 00 00 0A E8 03 00 00 F4 01 19 00 01
#   Decoded:
#     uint32[0]: 10
#     int16[0]: 1000
#     ...
```

This helps verify:
- Device is sending correct report ID (2)
- Data structure matches your schema
- Byte order is correct (little-endian)

### 8.2 Compare with ROS Topic

Compare raw bytes from `inspect_device` with values on ROS topic to verify conversion is working correctly.

## Step 9: Customize the Integration

### 9.1 Modify Controller Configuration

Edit the generated `config/controllers.yaml` to customize:
- Publishing rate
- Topic names
- Additional parameters

### 9.2 Add Custom Launch Arguments

Create your own launch file in `launch/my_launch.py` to:
- Add additional nodes
- Set parameters
- Configure logging

### 9.3 Create Custom Controllers

For advanced processing, create a custom controller (see [Custom Controllers](advanced/custom_controllers.md)).

## Troubleshooting

### Device Not Found

```bash
# Verify device is connected
lsusb | grep -i cafe

# Check udev rules
ls -l /etc/udev/rules.d/*accel*

# Test device access
ls -l /dev/hidraw*
```

### No Data on Topics

```bash
# Check controller status
ros2 control list_controllers

# View hardware component state
ros2 control list_hardware_components

# Enable debug logging
ros2 launch my_accel_bringup hid.launch.py --log-level debug
```

### Wrong Values

```bash
# Monitor raw USB and compare with topic
# Terminal 1:
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4001

# Terminal 2:
ros2 topic echo /accel_broadcaster/accel_states
```

**Common Issues:**
- Byte order: HID uses little-endian
- Struct packing: Firmware must use `__attribute__((packed))`
- Type mismatch: Schema type must match firmware

## Next Steps

### Integrate with Your Firmware

1. Copy the generated header to your firmware project:
   ```bash
   cp install/my_accel_bringup/include/my_accel_bringup/hid_descriptor.h \
      /path/to/firmware/include/
   ```

2. Use it in your firmware (see [Firmware Integration](advanced/firmware_integration.md))

### Add Command Interface

To send commands to your device:

1. Add `outputs` section to schema
2. Rebuild package
3. Create controller that writes to command interfaces

See [Signal Generator Example](examples/signal_generator.md) for bidirectional example.

### Process Data

Create a custom controller or node that:
- Subscribes to `/accel_broadcaster/accel_states`
- Processes accelerometer data
- Publishes IMU messages, odometry, etc.

## Summary

You've now:
- ✅ Set up HID ROS 2 from scratch
- ✅ Created a custom device package
- ✅ Defined a device schema
- ✅ Generated all necessary code automatically
- ✅ Launched and monitored your device
- ✅ Verified data flow from USB to ROS 2

The same workflow applies to any HID device - just modify the schema to match your device's report structure!

## Additional Resources

- [Schema Reference](schema_reference.md) - Complete YAML reference
- [Developer Tools](developer_tools.md) - Debugging and validation tools
- [Examples](examples/dummy_pose.md) - More complete examples
- [Troubleshooting](troubleshooting.md) - Common issues and solutions
