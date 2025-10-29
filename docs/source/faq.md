# Frequently Asked Questions

Common questions about HID ROS 2.

## General

### What is HID ROS 2?

A framework for integrating USB Human Interface Devices with ROS 2 using a schema-driven approach.

### Why use HID instead of serial/Ethernet?

- **Standard protocol**: No custom drivers needed
- **Plug-and-play**: OS handles enumeration
- **Bidirectional**: Input and output in one connection
- **Low latency**: Fast polling rates (up to 1000 Hz)

### What devices are supported?

Any USB HID-compliant device, including:
- Custom microcontroller projects
- Gaming peripherals
- Industrial sensors
- Input devices

## Schema and Configuration

### Can I change the schema after deployment?

Yes, but you'll need to rebuild and the firmware must match. The schema is the contract between firmware and ROS 2.

### Do I need to write URDF?

No! URDF is automatically generated from your YAML schema.

### Can I have multiple HID devices?

Yes, each gets its own hardware interface instance with different VID/PID.

## Development

### How do I debug my device?

Use the `inspect_device` tool:
```bash
ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP
```

### My values are wrong, what's the issue?

Common causes:
1. Byte order (use little-endian)
2. Wrong type in schema
3. Firmware struct not packed: `__attribute__((packed))`

### Can I use this with Arduino?

Yes! Use a HID-capable Arduino (Leonardo, Micro, etc.) and the generated HID descriptor.

## Advanced Usage

### Can I create custom controllers?

Yes! HID ROS 2 works with any ros2_control controller. See [Custom Controllers](advanced/custom_controllers.md).

### What's the maximum data rate?

Limited by USB HID polling rate (typically 1000 Hz max) and report size (typically 64 bytes max per report).

### Can I send commands to the device?

Yes! Define `outputs` in your schema to create command interfaces.

## More Questions?

Check the [Troubleshooting](troubleshooting.md) guide or ask in [GitHub Discussions](https://github.com/adnan-saood/hid_ros2/discussions).
