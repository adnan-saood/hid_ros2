# Hardware Interface

The `hid_hardware` package provides the ros2_control hardware interface plugin.

## Overview

`HidHardware` implements the `hardware_interface::SystemInterface` to bridge USB HID devices with ROS 2.

## Features

- USB HID communication via libhidapi
- Type-aware data conversion
- Bidirectional communication support
- Automatic device reconnection
- Error handling and diagnostics

## Configuration

The hardware interface is configured through URDF parameters generated from your schema:

```xml
<hardware>
  <plugin>hid_hardware/HidHardware</plugin>
  <param name="vendor_id">0x046d</param>
  <param name="product_id">0xc07e</param>
  <param name="input_report_id">2</param>
  <param name="output_report_id">1</param>
</hardware>
```

## Lifecycle

Follows standard ros2_control lifecycle states.

## Implementation Details

For implementation details, see the source code in `hid_hardware/src/hid_hardware.cpp`.
