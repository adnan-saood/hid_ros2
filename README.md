# HID ROS 2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](https://github.com/adnan-saood/hid_ros2/actions/workflows/ci.yaml/badge.svg)](https://github.com/adnan-saood/hid_ros2/actions/workflows/ci.yaml)

**A schema-driven ROS 2 hardware interface for USB Human Interface Devices (HID)**

**A schema-driven ROS 2 hardware interface for USB Human Interface Devices (HID)**

## Overview

HID ROS 2 is a comprehensive framework for integrating USB HID devices with ROS 2 through the `ros2_control` infrastructure. It provides a schema-driven approach where a single YAML file defines your device's interface, automatically generating all necessary code for both the ROS 2 side and embedded firmware.

### Key Features

- **Single Source of Truth**: Define your device interface once in YAML, generate everything else automatically
- **Type-Aware Hardware Interface**: Native support for `uint8/16/32`, `int8/16/32`, and `float32/64` with proper byte conversion
- **Bidirectional Communication**: Both input (sensor data) and output (commands) over USB HID
- **Code Generation**: Automatic generation of URDF, controller configs, launch files, and C headers for firmware
- **Developer Tools**: Command-line tools for schema validation, live device inspection, and descriptor generation
- **Full ros2_control Integration**: Works seamlessly with existing ROS 2 controllers and tools
- **Zero Manual Configuration**: No hand-written URDF or configuration files required

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         YAML Schema                              │
│                    (Single Source of Truth)                      │
└──────────────────────┬──────────────────────────────────────────┘
                       │
          ┌────────────┴────────────┐
          │                         │
          ▼                         ▼
┌─────────────────────┐   ┌─────────────────────┐
│   ROS 2 Side        │   │  Firmware Side      │
│  (Auto-Generated)   │   │  (Auto-Generated)   │
│                     │   │                     │
│  • URDF/Xacro       │   │  • HID Descriptor   │
│  • Controllers.yaml │   │  • Report Structs   │
│  • Launch files     │   │  • Type-safe API    │
└─────────────────────┘   └─────────────────────┘
          │                         │
          │    ┌──────────┐         │
          └───▶│ USB HID  │◀────────┘
               │ Protocol │
               └──────────┘
```

## Quick Start

### Prerequisites

- ROS 2 Humble or later
- Ubuntu 22.04 or compatible Linux distribution
- USB HID device (or develop with provided examples)

### Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/adnan-saood/hid_ros2.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Basic Usage

1. **Define your device schema** (`schema/my_device.yaml`):

```yaml
device_name: "my_sensor"
vendor_id: "0x046d"
product_id: "0xc07e"
input_report_id: 2
output_report_id: 1

sensor_name: "sensor_input"
frame_id: "sensor_frame"
update_rate: 250

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
```

2. **Configure your package** (add to `CMakeLists.txt`):

```cmake
find_package(hid_descriptor_generator REQUIRED)
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

hid_generate(SCHEMA_FILE "schema/my_device.yaml")
```

3. **Build and launch**:

```bash
colcon build --packages-select my_package
ros2 launch my_package hid.launch.py
```

That's it! The framework generates all URDF, configuration, launch files, and firmware headers automatically.

## Package Structure

This repository contains several packages working together:

### Core Packages

- **`hid_hardware`**: Hardware interface plugin implementing `hardware_interface::SystemInterface`
  - Manages USB HID communication via `hidapi`
  - Handles bidirectional data transfer with configurable report IDs
  - Provides type-aware state/command interfaces to ros2_control

- **`hid_descriptor_generator`**: Code generation from YAML schemas
  - Generates URDF with correct state/command interfaces
  - Creates controller configuration files
  - Produces launch files with TF broadcasting
  - Generates C headers for embedded firmware

- **`hid_tools`**: Developer utilities for debugging and validation
  - `validate_schema`: YAML schema validator with strict mode
  - `inspect_device`: Live USB HID device monitor and debugger
  - `generate_hid_descriptor`: Standalone HID descriptor generator

### Controller Packages

- **`hid_generic_broadcaster`**: Generic sensor data broadcaster
  - Publishes all state interfaces as ROS 2 topics
  - Type-aware message generation
  - Configurable publishing rates

- **`hid_diagnostic_broadcaster`**: Device health monitoring
  - Connection status tracking
  - Error rate monitoring
  - Diagnostic message publishing

- **`signal_command_controller`**: Example command controller
  - Demonstrates bidirectional communication
  - Sends commands to HID device via output reports
  - Suitable for actuators or configurable sensors

### Example Packages

- **`examples/dummy_pose_device`**: Complete working example
  - Simulated 3-DOF pose sensor
  - Reference implementation for new devices

- **`examples/signal_generator`**: Bidirectional example
  - Demonstrates both input and output reports
  - Configurable signal generation parameters

## Documentation

- **[QUICKSTART.md](QUICKSTART.md)**: Comprehensive quick-start guide with examples
- **[CONTRIBUTING.md](CONTRIBUTING.md)**: Development guidelines and contribution workflow
- **[hid_tools/README.md](hid_tools/README.md)**: Complete reference for developer tools
- **[hid_descriptor_generator/README.md](hid_descriptor_generator/README.md)**: Code generation documentation

## Supported Data Types

The framework provides native support for multiple data types with automatic byte conversion:

| Type      | Size (bits) | Signed | Use Case                    |
|-----------|-------------|--------|-----------------------------|
| `uint8`   | 8           | No     | Bytes, small unsigned       |
| `int8`    | 8           | Yes    | Small signed values         |
| `uint16`  | 16          | No     | Larger unsigned values      |
| `int16`   | 16          | Yes    | Sensor readings             |
| `uint32`  | 32          | No     | Timestamps, large unsigned  |
| `int32`   | 32          | Yes    | Large signed values         |
| `float32` | 32          | Yes    | Floating-point measurements |
| `float64` | 64          | Yes    | High-precision values       |

Arrays are supported via the `count` field in YAML, creating indexed interfaces (e.g., `accel_0`, `accel_1`, `accel_2`).

## Development Workflow

### 1. Schema Development

```bash
# Create and validate schema
vim my_device.yaml
ros2 run hid_tools validate_schema my_device.yaml --strict

# Generate firmware descriptor
ros2 run hid_tools generate_hid_descriptor my_device.yaml -o firmware/hid_desc.h
```

### 2. Firmware Development

Use the generated C header in your microcontroller code:

```c
#include "my_package/hid_descriptor.h"

// USB HID initialization
const uint8_t* desc = hid_report_descriptor;
uint16_t desc_len = HID_REPORT_DESCRIPTOR_SIZE;

// Type-safe report structure
HIDInputReport report = {
    .report_id = HID_INPUT_REPORT_ID,
    .position_x = 1.5f,
    .position_y = 2.3f,
    .velocity = 150
};

// Send to host
hid_send((uint8_t*)&report, sizeof(report));
```

### 3. ROS 2 Integration

```bash
# Build with code generation
colcon build --packages-select my_package

# Launch system
ros2 launch my_package hid.launch.py

# Monitor state interfaces
ros2 control list_hardware_interfaces
ros2 topic echo /sensor_input_states
```

### 4. Live Debugging

```bash
# List connected HID devices
ros2 run hid_tools inspect_device --list

# Monitor raw USB reports
ros2 run hid_tools inspect_device --vid 0x046d --pid 0xc07e

# Send test commands
ros2 run hid_tools inspect_device --vid 0x046d --pid 0xc07e \
  --send 1:0x00,0x00,0x80,0x3F
```

## Testing

The project includes comprehensive testing infrastructure:

```bash
# Run all tests
colcon test

# View test results
colcon test-result --all --verbose

# Run tests with coverage
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage"
colcon test
```

## Continuous Integration

The repository includes GitHub Actions workflows for:

- **Linting**: Pre-commit hooks, cpplint, pydocstyle, ament tools
- **Multi-Distribution Testing**: Automated testing on ROS 2 Humble and Iron
- **Code Coverage**: Coverage reporting with gcovr and Codecov integration

All contributions are automatically validated against these checks.

## Hardware Requirements

### Supported Devices

Any USB HID-compliant device can be used, including:

- Custom microcontroller-based sensors (ESP32, STM32, etc.)
- Gaming peripherals (mice, joysticks, game controllers)
- Industrial sensors with HID interfaces
- Custom input devices

### USB Permissions

On Linux, you may need to configure udev rules for device access:

```bash
# Create udev rule
sudo tee /etc/udev/rules.d/99-hid-device.rules <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c07e", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Replace `046d` and `c07e` with your device's vendor ID and product ID.

## Troubleshooting

### Device Not Found

```bash
# Check device connection
lsusb

# Verify vendor/product IDs in YAML match device
ros2 run hid_tools inspect_device --list

# Check permissions
ls -l /dev/hidraw*
```

### Wrong Data Values

```bash
# Compare raw USB data with ROS 2 topics
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000
ros2 topic echo /sensor_input_states

# Verify report structure in firmware matches schema
# Check byte packing with __attribute__((packed))
```

### Build Errors

```bash
# Ensure code generator is built first
colcon build --packages-select hid_descriptor_generator
colcon build --packages-select my_package

# Check YAML schema is valid
ros2 run hid_tools validate_schema schema/my_device.yaml --strict
```

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for:

- Development setup and workflow
- Coding standards and style guides
- Testing requirements
- Pull request process

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

## Acknowledgments

Built on the robust `ros2_control` framework and designed for seamless integration with the ROS 2 ecosystem.

## Support

- **Issues**: [GitHub Issues](https://github.com/adnan-saood/hid_ros2/issues)
- **Discussions**: [GitHub Discussions](https://github.com/adnan-saood/hid_ros2/discussions)

---

**Maintainer**: Adnan Saood (adnan.saood@outlook.com)

# How to use this Package and ROS Introduction

- [HID ROS 2](#hid-ros-2)
  - [Overview](#overview)
    - [Key Features](#key-features)
    - [Architecture](#architecture)
  - [Quick Start](#quick-start)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
    - [Basic Usage](#basic-usage)
  - [Package Structure](#package-structure)
    - [Core Packages](#core-packages)
    - [Controller Packages](#controller-packages)
    - [Example Packages](#example-packages)
  - [Documentation](#documentation)
  - [Supported Data Types](#supported-data-types)
  - [Development Workflow](#development-workflow)
    - [1. Schema Development](#1-schema-development)
    - [2. Firmware Development](#2-firmware-development)
    - [3. ROS 2 Integration](#3-ros-2-integration)
    - [4. Live Debugging](#4-live-debugging)
  - [Testing](#testing)
  - [Continuous Integration](#continuous-integration)
  - [Hardware Requirements](#hardware-requirements)
    - [Supported Devices](#supported-devices)
    - [USB Permissions](#usb-permissions)
  - [Troubleshooting](#troubleshooting)
    - [Device Not Found](#device-not-found)
    - [Wrong Data Values](#wrong-data-values)
    - [Build Errors](#build-errors)
  - [Contributing](#contributing)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)
  - [Support](#support)
- [How to use this Package and ROS Introduction](#how-to-use-this-package-and-ros-introduction)
  - [Workflow With Docker](#workflow-with-docker)
    - [Quick Start Using ROS with Docker (ros\_team\_workspace)](#quick-start-using-ros-with-docker-ros_team_workspace)
  - [Install and Build](#install-and-build)
    - [Install ROS Humble and Development Tooling](#install-ros-humble-and-development-tooling)
    - [Setup ROS Workspace](#setup-ros-workspace)
    - [Configure and Build Workspace:](#configure-and-build-workspace)
  - [Running Executable](#running-executable)
    - [Using the Local Workspace](#using-the-local-workspace)
      - [Notes on Sourcing ROS Workspace](#notes-on-sourcing-ros-workspace)
  - [Testing and Linting](#testing-and-linting)
  - [Creating a new ROS 2 Package](#creating-a-new-ros-2-package)
  - [References](#references)

## Workflow With [Docker](https://docs.docker.com/)

> **NOTE:** If you do not use Docker in the current workflow you can skip this section and jump to [Install and Build](#install-and-build)

We usually use a separate [Docker](https://docs.docker.com/) container for each of the projects/workspaces we work on.
An internal tool from [Stogl Robotics](https://b-robotized.com) called [Ros Team Workspace (RTW)](https://rtw.b-robotized.com) simplifies the creation and work with  Docker based workspaces.
The tool is targeted toward developers.

Installation of docker depends on the operating system you are using. Instructions can be found here: [Windows](https://docs.docker.com/desktop/install/windows-install/), [Mac](https://docs.docker.com/desktop/install/mac-install/) and [Linux](https://docs.docker.com/desktop/install/linux-install/).

### Quick Start Using ROS with Docker (ros_team_workspace)

Using [Ros Team Workspace (RTW)](https://rtw.b-robotized.com) you can easily with the following command:
```
setup-ros-workspace-docker WS_FOLDER_NAME ROS_DISTRO
```
and then after sourcing the new workspace with the `_WS_FOLDER_NAME` command, you can switch to the workspace with the:
```
rtw_switch_to_docker
```
command.

## Install and Build

### Install ROS Humble and Development Tooling

These instructions assume you are running Ubuntu 20.04:

1. [Install ROS 2 Humble](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Install-Debians/).
   You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Install-Debians/#environment-setup).
   Make sure you setup your environment with:
   ```
   source /opt/ros/humble/setup.bash
   ```

   > **NOTE:** You may want to add that line to your `~/.bashrc`

   > **NOTE:** There is also a `zsh` version of the setup script.

1. [Install ROS 2 Build Tools](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Development-Setup/#install-development-tools-and-ros-tools).
   You do not need to build ROS 2 from source.
   Simply install the tooling under the section titled "Install development tools and ROS tools".

1. Install `ccache`:
   ```
   sudo apt install ccache
   ```

1. Setup `colcon mixin` [Reference](https://github.com/colcon/colcon-mixin-repository) for convenience commands.
   ```
   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default
   ```

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_humble
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspace/ros_ws_humble` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ros_ws_humble`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:hid_hardware/hid_hardware.git src/hid_hardware
   vcs import src --input src/hid_hardware/hid_hardware.humble.repos
   vcs import src --input src/hid_hardware/hid_hardware.humble.repos
   rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
   ```

   Sometimes packages do not list all their dependencies so `rosdep` will not install everything.
   If you are getting missing dependency errors, try manually install the following packages:
   ```
   sudo apt install ros2-humble-forward_command_controller ros2-humble-joint_state_broadcaster ros2-humble-joint_trajectory_controller ros2-humble-xacro
   ```

### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Executable

See `README.md` files of the packages for information regarding running executables.

<Add here some concrete data about current repository>

### Using the Local Workspace

To use the local workspace you have to source it by using local setup script:
  ```
  source $COLCON_WS/install/local_setup.bash
  ```
Since there are many errors one unintentionally do with wrong sourcing, please check also [Notes on Sourcing ROS Workspace](#notes-on-sourcing-ros-workspace).

#### Notes on Sourcing ROS Workspace

Sourcing of a workspace appends the binary and resource directories to appropriate environment variables.
It is important that you do not run the build command in the same terminal that you have previously sourced your local workspace.
This can cause dependency resolution issues.
Here is some advice copied from [Official ROS Workspace Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) on this:

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace.
Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment.
`setup` sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup, like you just did, is the same as just sourcing dev_ws’s setup, because that includes the environment of the underlay it was created in.


## Testing and Linting

To test the packages packages built from source, use the following command with [colcon](https://colcon.readthedocs.io/en/released/).
In order to run tests and linters you will have had to already built the workspace.
To run the tests use following commands:
  ```
  cd $COLCON_WS
  colcon test
  colcon test-result
  ```

There are `--mixin` arguments that can be used to control testing with linters, specifically `linters-only` and `linters-skip`.

## Creating a new ROS 2 Package

If you need to create a new ROS 2 package it is helpful to start with the official boilerplate for a ROS 2 package.
The command `ros2 pkg` can be used to generate the boilerplate details.
For example to create a new ROS 2 package called `example_package` with a node called `example_node` and library called `example_library` use this command:
  ```
  ros2 pkg create --build-type ament_cmake --node-name example_node --library-name example_library example_package
  ```

## References

Here are some useful references for developing with ROS 2:

 - [Official ROS 2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
   * [Launchfile](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
   * [Package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * [Parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/)
   * [Workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
 - [Example ROS packages](https://github.com/ros2/examples)
 - [Colcon Documentation](https://colcon.readthedocs.io/en/released/#)
 - [ROS 2 Design Documentation](https://design.ros2.org/)
 - [ROS 2 Launch Architecture](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)

Pluginlib-Library: hid_hardware
Plugin: hid_hardware/HidHardware (hardware_interface::SystemInterface)
