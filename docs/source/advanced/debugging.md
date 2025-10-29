# Debugging

Advanced debugging techniques for HID ROS 2.

## Debugging Tools

### 1. inspect_device

The primary tool for debugging USB communication:

```bash
# Monitor raw reports
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000
```

Shows:
- Report IDs
- Raw bytes
- Decoded values (uint32, float32, etc.)
- Update rate statistics

### 2. ros2 control CLI

Check hardware interface status:

```bash
# List all interfaces
ros2 control list_hardware_interfaces

# List controllers
ros2 control list_controllers

# Controller status
ros2 control view_controller_chains
```

### 3. ROS 2 Topic Tools

Monitor published data:

```bash
# Echo topic
ros2 topic echo /hid_sensor_states

# Topic info
ros2 topic info /hid_sensor_states -v

# Monitor rate
ros2 topic hz /hid_sensor_states
```

## Common Issues

### No Data Received

**Check 1**: Device connected and recognized
```bash
lsusb | grep CAFE
ros2 run hid_tools inspect_device --list
```

**Check 2**: Correct report ID
```bash
# Compare firmware report ID with schema
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000
```

**Check 3**: Hardware interface active
```bash
ros2 control list_hardware_interfaces
# Should show [available] [claimed]
```

### Wrong Values

**Check 1**: Byte order
```bash
# View raw bytes
ros2 run hid_tools inspect_device --vid 0xCAFE --pid 0x4000
```

Verify little-endian format.

**Check 2**: Type mismatch
- Schema says `float32` but firmware sends `int16`?
- Check struct in firmware matches schema

**Check 3**: Struct packing
```c
// Firmware must use packed structs
typedef struct __attribute__((packed)) {
    // ...
} Report;
```

Verify with:
```c
printf("Size: %zu\n", sizeof(Report));
```

### Performance Issues

**Check 1**: Update rate too high
```bash
# Monitor actual rate
ros2 topic hz /hid_sensor_states
```

Reduce `update_rate` in schema if dropping data.

**Check 2**: USB bandwidth
- Full-speed USB: ~12 Mbps
- Calculate: `report_size * update_rate * 8`
- Should be well below 12 Mbps

**Check 3**: Processing time
Add timing to controller:
```cpp
auto start = std::chrono::high_resolution_clock::now();
// ... update code ...
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "Update took %ld μs", duration.count());
```

## Debugging Workflow

1. **Verify device basics**
   ```bash
   lsusb
   ros2 run hid_tools inspect_device --list
   ```

2. **Check raw USB communication**
   ```bash
   ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP
   ```

3. **Verify hardware interface**
   ```bash
   ros2 control list_hardware_interfaces
   ```

4. **Check controller status**
   ```bash
   ros2 control list_controllers
   ```

5. **Monitor published data**
   ```bash
   ros2 topic echo /sensor_states
   ```

## Advanced Debugging

### USB Packet Capture

Use Wireshark with usbmon:

```bash
# Load usbmon kernel module
sudo modprobe usbmon

# Capture with Wireshark
wireshark
# Select usbmonX interface, filter: usb.device_address == YOUR_DEVICE
```

### Firmware Debugging

Add debug output in firmware:

```c
void send_report(void) {
    HIDInputReport report = { /* ... */ };

    // Debug print
    printf("Sending report ID=%d, x=%.2f, y=%.2f\n",
           report.report_id, report.x, report.y);

    usb_hid_send((uint8_t*)&report, sizeof(report));
}
```

Compare with `inspect_device` output.

### GDB Debugging

Debug the hardware interface:

```bash
# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run with GDB
ros2 run --prefix 'gdb --args' hid_hardware hid_hardware
```

Set breakpoints in `read()` and `write()` methods.

### Logging

Enable debug logging:

```bash
ros2 launch my_package hid.launch.py --log-level debug
```

Or in code:
```cpp
RCLCPP_DEBUG(get_logger(), "Read value: %f", value);
```

## Validation Tools

### Schema Validation

```bash
ros2 run hid_tools validate_schema --strict schema/hid_device.yaml
```

### Report Size Calculation

```python
# Calculate expected report size
from schema_parser import parse_schema

schema = parse_schema("schema/hid_device.yaml")
input_size = calculate_report_size(schema['fields'])
print(f"Expected input report size: {input_size + 1} bytes (including report ID)")
```

Compare with actual size from inspect_device.

## Getting Help

If stuck:

1. Check [Troubleshooting](../troubleshooting.md)
2. Search [GitHub Issues](https://github.com/adnan-saood/hid_ros2/issues)
3. Ask in [GitHub Discussions](https://github.com/adnan-saood/hid_ros2/discussions)

When reporting issues, include:
- ROS 2 distribution and version
- Schema YAML
- Output from `inspect_device`
- Output from `list_hardware_interfaces`
- Relevant logs
