# Troubleshooting

Common issues and their solutions.

## Device Connection Issues

### Device Not Found

**Symptoms:**
```
[ERROR] [hid_hardware]: Failed to open device 046d:c07e
```

**Solutions:**

1. **Verify device is connected:**
   ```bash
   lsusb | grep 046d:c07e
   ```

2. **Check VID/PID in schema:**
   Compare with actual device IDs from `lsusb`

3. **List available HID devices:**
   ```bash
   ros2 run hid_tools inspect_device --list
   ```

### Permission Denied

**Symptoms:**
```
[ERROR] [hid_hardware]: Permission denied accessing HID device
```

**Solution:**

Set up udev rules (see [Installation](installation.md#setting-up-usb-permissions)):
```bash
sudo tee /etc/udev/rules.d/99-hid-devices.rules <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Build Issues

### Generator Not Found

**Symptoms:**
```
CMake Error: Could not find a package configuration file provided by "hid_descriptor_generator"
```

**Solution:**

Build packages in order:
```bash
colcon build --packages-select hid_descriptor_generator
colcon build
```

### Python Import Errors

**Symptoms:**
```
ModuleNotFoundError: No module named 'yaml'
```

**Solution:**
```bash
pip install pyyaml hidapi
```

## Runtime Issues

### No Data on Topics

**Symptoms:**
Topics exist but show no data

**Debugging:**

1. **Check hardware interface state:**
   ```bash
   ros2 control list_hardware_interfaces
   ```

2. **Monitor raw USB data:**
   ```bash
   ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP
   ```

3. **Verify device is sending data:**
   If `inspect_device` shows no data, problem is in firmware

### Wrong Values

**Symptoms:**
Values on topics don't match expected

**Debugging:**

Compare raw bytes with published values:
```bash
# Terminal 1: Raw USB
ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP

# Terminal 2: ROS topics
ros2 topic echo /sensor_states
```

**Common causes:**
- Byte order mismatch (use little-endian)
- Wrong type in schema
- Firmware struct not `__attribute__((packed))`

## Getting Help

- **GitHub Issues**: Report bugs
- **Discussions**: Ask questions
- **Documentation**: Check guides first

When reporting issues, include:
- ROS 2 distribution
- Schema YAML file
- Error messages
- Output from `ros2 control list_hardware_interfaces`
