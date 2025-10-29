# Type System

Deep dive into the type-aware data handling in HID ROS 2.

## Overview

HID ROS 2 provides native support for multiple data types with automatic byte-level conversion between USB HID reports and ros2_control interfaces.

## Type Conversions

### Integer Types

All integer types use little-endian byte order:

```cpp
// uint16 example
uint8_t bytes[2] = {0x34, 0x12};  // From USB
uint16_t value = bytes[0] | (bytes[1] << 8);  // = 0x1234 = 4660

// int16 example (two's complement)
uint8_t bytes[2] = {0xFF, 0xFF};  // From USB
int16_t value = bytes[0] | (bytes[1] << 8);  // = -1
```

### Floating-Point Types

IEEE 754 format, little-endian:

```cpp
// float32 example
uint8_t bytes[4] = {0x00, 0x00, 0x80, 0x3F};  // From USB
float value;
memcpy(&value, bytes, 4);  // = 1.0

// To ros2_control interface (always double)
double interface_value = static_cast<double>(value);
```

## Byte Packing

### Report Structure

Reports are tightly packed with no padding:

```yaml
fields:
  - name: "a"
    type: "uint8"   # 1 byte
  - name: "b"
    type: "float32" # 4 bytes
  - name: "c"
    type: "int16"   # 2 bytes
# Total: 7 bytes (plus 1 byte report ID = 8 bytes)
```

### Firmware Struct

```c
typedef struct __attribute__((packed)) {
    uint8_t report_id;  // Offset 0
    uint8_t a;          // Offset 1
    float b;            // Offset 2 (no alignment padding!)
    int16_t c;          // Offset 6
} Report;  // Total: 8 bytes
```

**Critical**: Use `__attribute__((packed))` to prevent compiler padding.

## ros2_control Interface

All interfaces use `double` regardless of underlying type:

```cpp
// State interface (read)
double value = state_interfaces_[0].get_value();

// Command interface (write)
command_interfaces_[0].set_value(3.14159);
```

This ensures compatibility with all controllers.

## Type Safety

### Schema Validation

The validator checks type consistency:
- Valid type names
- Reasonable array sizes
- Report size limits

### Runtime Checks

The hardware interface validates:
- Report sizes match schema
- Report IDs are correct
- Conversion doesn't overflow

## Performance

### Optimization

Type-aware conversion is optimized:
- Direct memory copies for native types
- Minimal byte manipulation
- Zero-copy where possible

### Latency

Typical latency for a 4-field report:
- USB transfer: ~1 ms
- Parsing: <10 μs
- ros2_control update: ~100 μs

Total: ~1.1 ms (suitable for 1 kHz control loops)

## Platform Considerations

### Endianness

HID ROS 2 assumes little-endian (x86, ARM):
- Works on most modern platforms
- May need modification for big-endian

### Alignment

Reports are packed, but modern CPUs handle unaligned access efficiently.

### Floating-Point

Both firmware and ROS 2 must use IEEE 754:
- Standard on ARM, x86
- May differ on exotic platforms

## Advanced Topics

### Custom Types

For custom types, modify the hardware interface:

```cpp
// Example: 24-bit signed integer
int32_t read_int24(const uint8_t* bytes) {
    int32_t value = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16);
    // Sign extend
    if (value & 0x800000) value |= 0xFF000000;
    return value;
}
```

### Compressed Data

For bandwidth-critical applications:
- Use smaller types (int16 instead of float32)
- Scale values in firmware
- Descale in controller

Example:
```yaml
# Instead of float32 temperature in °C
- name: "temp"
  type: "int16"  # Temperature in 0.01°C
```

Firmware: `report.temp = (int16_t)(temperature * 100.0f);`

Controller: `double temp_celsius = value / 100.0;`

## Debugging Type Issues

### Verify Byte Order

```bash
# Monitor raw bytes
ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP
```

Look for pattern in bytes to verify endianness.

### Check Alignment

```c
// In firmware, verify struct size
_Static_assert(sizeof(HIDInputReport) == EXPECTED_SIZE, "Struct size mismatch");
```

### Test Conversion

Create unit tests for type conversion:

```cpp
TEST(TypeConversion, Float32) {
    uint8_t bytes[4] = {0x00, 0x00, 0x80, 0x3F};
    float result = convert_float32(bytes);
    EXPECT_FLOAT_EQ(result, 1.0f);
}
```

For more examples, see the test directory in `hid_hardware`.
