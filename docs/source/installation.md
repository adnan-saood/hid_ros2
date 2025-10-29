# Installation

This guide covers installation of HID ROS 2 on Ubuntu Linux with ROS 2 Humble or later.

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) or later
- **ROS 2 Distribution**: Humble Hawksbill or later
- **Python**: 3.10 or later
- **Build Tools**: colcon, CMake 3.16+

### Required Dependencies

- ROS 2 base installation
- `ros2_control` and `controller_manager`
- `hidapi` library for USB HID communication
- Python dependencies: `pyyaml`

## Installing ROS 2

If you don't have ROS 2 installed, follow the official installation guide:

1. **Set up sources**:
   ```bash
   sudo apt update && sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 Humble**:
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-humble-desktop
   ```

3. **Set up environment**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Installing Build Tools

Install development tools required for building ROS 2 packages:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

## Installing HID ROS 2

### From Source

1. **Create a workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the repository**:
   ```bash
   git clone https://github.com/adnan-saood/hid_ros2.git
   ```

3. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the workspace**:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

5. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

   Add this to your `~/.bashrc` for convenience:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

## Installing USB HID Libraries

The hardware interface requires `libhidapi-dev` for USB communication:

```bash
sudo apt install libhidapi-dev libhidapi-hidraw0 libhidapi-libusb0
```

## Installing Python Dependencies

For the developer tools and code generator:

```bash
pip install pyyaml hidapi
```

## Verifying Installation

1. **Check package installation**:
   ```bash
   ros2 pkg list | grep hid
   ```

   You should see:
   ```
   hid_descriptor_generator
   hid_diagnostic_broadcaster
   hid_generic_broadcaster
   hid_hardware
   hid_tools
   signal_command_controller
   ```

2. **Verify developer tools**:
   ```bash
   ros2 run hid_tools validate_schema --help
   ros2 run hid_tools inspect_device --help
   ros2 run hid_tools generate_hid_descriptor --help
   ```

3. **Check hardware interface plugin**:
   ```bash
   ros2 control list_hardware_interfaces
   ```

(setting-up-usb-permissions)=
## Setting Up USB Permissions

To access HID devices without root privileges, create a udev rule:

1. **Create udev rule file**:
   ```bash
   sudo nano /etc/udev/rules.d/99-hid-devices.rules
   ```

2. **Add rules** (replace VID/PID with your device):
   ```
   # Generic HID devices - adjust vendor_id and product_id as needed
   SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c07e", MODE="0666", GROUP="plugdev"
   SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c07e", MODE="0666", GROUP="plugdev"
   ```

3. **Reload udev rules**:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Add user to plugdev group**:
   ```bash
   sudo usermod -a -G plugdev $USER
   ```

   Log out and log back in for group changes to take effect.

## Optional: Install Pre-commit Hooks

For development, install pre-commit hooks:

```bash
pip install pre-commit
cd ~/ros2_ws/src/hid_ros2
pre-commit install
```

## Next Steps

Now that you have HID ROS 2 installed, proceed to the [Quick Start Guide](quickstart.md) to create your first HID device integration.

## Troubleshooting

### Build Failures

**Problem**: `Could not find a package configuration file provided by "hid_descriptor_generator"`

**Solution**: Build packages in the correct order:
```bash
colcon build --packages-select hid_descriptor_generator
colcon build
```

### USB Permission Denied

**Problem**: `Failed to open HID device: Permission denied`

**Solution**:
1. Check udev rules are correctly set up
2. Verify user is in `plugdev` group: `groups $USER`
3. Reconnect the device or reboot

### Missing Python Modules

**Problem**: `ModuleNotFoundError: No module named 'yaml'`

**Solution**: Install Python dependencies:
```bash
pip install pyyaml hidapi
```

For system-wide installation:
```bash
sudo apt install python3-yaml python3-hidapi
```
