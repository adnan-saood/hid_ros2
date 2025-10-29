# Dummy Pose Device Example

Complete example of a 6-DOF pose sensor (position + rotation).

## Overview

The dummy_pose_device example demonstrates:
- Complete YAML schema
- Generated code integration
- Custom controller
- Launch configuration

## Schema

Located at `examples/dummy_pose_device/dummy_bringup/schema/hid_device.yaml`:

```yaml
device_name: "dummy_hid_device"
vendor_id: "0xCAFE"
product_id: "0x4000"
report_id: 1

sensor_name: "hid_input"
frame_id: "dummy_hid_frame"
update_rate: 250

fields:
  - name: "x"
    type: "uint8"
    description: "X position"
  - name: "y"
    type: "uint8"
    description: "Y position"
  - name: "z"
    type: "uint8"
    description: "Z position"
  - name: "rx"
    type: "uint8"
    description: "X rotation"
  - name: "ry"
    type: "uint8"
    description: "Y rotation"
  - name: "rz"
    type: "uint8"
    description: "Z rotation"
```

## Building

```bash
colcon build --packages-select dummy_pose_broadcaster dummy_bringup
```

## Running

```bash
ros2 launch dummy_bringup dummy.launch.py
```

## Source Code

See the example package for complete implementation details.
