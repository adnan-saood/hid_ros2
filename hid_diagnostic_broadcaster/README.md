# HID Diagnostic Broadcaster

A ROS2 controller that publishes diagnostic information about HID devices to the `/diagnostics` topic.

## Features

- **Connection Status**: Reports if device is connected and responding
- **Read Rate Monitoring**: Tracks actual HID read frequency
- **Device Information**: Publishes VID, PID, and device name
- **Data Health**: Detects frozen/stuck data
- **Current Values**: Shows first 10 interface values in diagnostics
- **Integration**: Works with `rqt_robot_monitor` and diagnostic aggregators

## Published Topics

- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) - Device health and status at 1Hz

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_name` | string | `"hid_input"` | Name of the sensor to monitor |
| `publish_rate` | double | `1.0` | Diagnostic publish rate in Hz |
| `device_name` | string | `"Unknown HID Device"` | Human-readable device name |
| `vendor_id` | string | `"0x0000"` | USB Vendor ID |
| `product_id` | string | `"0x0000"` | USB Product ID |

## Usage

### Enable in Launch File

Uncomment the spawner in your launch file:

```python
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["hid_diagnostic_broadcaster", "--controller-manager", "/controller_manager"],
),
```

### View Diagnostics

**Using rqt:**
```bash
rqt_robot_monitor
```

**Command line:**
```bash
ros2 topic echo /diagnostics
```

## Diagnostic Status Levels

| Level | Condition |
|-------|-----------|
| **OK** | Device responding, data changing, read rate > 10Hz |
| **WARN** | Low read rate (<10Hz) or data frozen (>1s unchanged) |
| **ERROR** | No state interfaces available |

## Reported Metrics

- Device Name, VID, PID
- Read Rate (Hz)
- Total Reads
- Interface Count
- Time Since Last Change
- Current values for all interfaces (first 10 shown)

## Optional Feature

This broadcaster is **optional** - the system works without it. Enable only when you need device health monitoring or debugging.
