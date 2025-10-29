# Firmware Integration

Guide to integrating generated HID descriptors with microcontroller firmware.

## Overview

This guide covers using the generated C header files in your embedded firmware.

## Supported Platforms

The generated code is platform-independent C and works with:
- Arduino (Leonardo, Micro, Due, etc.)
- ESP32
- STM32
- Any microcontroller with USB HID support

## Using the Generated Header

### Include the Header

```c
#include "my_package/hid_descriptor.h"
```

### USB HID Descriptor

Use the generated descriptor for USB enumeration:

```c
// In your USB HID initialization
const uint8_t* get_hid_descriptor(uint16_t* length) {
    *length = HID_REPORT_DESCRIPTOR_SIZE;
    return hid_report_descriptor;
}
```

### Report Structures

Use the type-safe structures:

```c
// Input report (sensor data to host)
HIDInputReport input_report = {
    .report_id = HID_INPUT_REPORT_ID,
    .position_x = 1.5f,
    .position_y = 2.3f,
    .velocity = 150
};

// Send to host
usb_hid_send((uint8_t*)&input_report, sizeof(input_report));
```

### Receiving Output Reports

```c
// Receive from host
uint8_t buffer[64];
int len = usb_hid_receive(buffer, sizeof(buffer));

if (len > 0 && buffer[0] == HID_OUTPUT_REPORT_ID) {
    HIDOutputReport* output = (HIDOutputReport*)buffer;

    // Process commands
    set_led_brightness(output->led_brightness);
    set_motor_speed(output->motor_speed);
}
```

## Platform-Specific Examples

### Arduino

```cpp
#include "HID.h"
#include "my_package/hid_descriptor.h"

static const uint8_t _hidReportDescriptor[] PROGMEM = {
    // Copy from generated hid_report_descriptor
};

void setup() {
    static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
    HID().AppendDescriptor(&node);
}

void loop() {
    HIDInputReport report = { /* ... */ };
    HID().SendReport(HID_INPUT_REPORT_ID, &report, sizeof(report));
    delay(10);
}
```

### ESP32

```c
#include "my_package/hid_descriptor.h"
#include "esp_hid_device.h"

// Use generated descriptor
const uint8_t* desc = hid_report_descriptor;
uint16_t desc_len = HID_REPORT_DESCRIPTOR_SIZE;

// Initialize ESP32 HID
esp_hid_device_config_t config = {
    .vendor_id = 0xCAFE,
    .product_id = 0x4000,
    .report_maps = &desc,
    .report_maps_len = desc_len
};

esp_hid_device_init(&config);
```

## Important Considerations

### Structure Packing

Always use `__attribute__((packed))` (already in generated code):

```c
typedef struct __attribute__((packed)) {
    uint8_t report_id;
    float value;
} HIDInputReport;
```

### Byte Order

Use little-endian (standard for USB HID). The generated structures handle this automatically on little-endian platforms (ARM, x86).

### Report Size

Keep reports under 64 bytes for maximum compatibility:
- Full-speed USB: 64 bytes max
- Low-speed USB: 8 bytes max

### Timing

Match the update rate in your firmware to the `update_rate` in the schema.

## Debugging

### Verify Descriptor

Use Linux `usbhid-dump` to verify your device's descriptor:

```bash
sudo usbhid-dump -e descriptor -m VID:PID
```

Compare with generated descriptor.

### Test with inspect_device

```bash
ros2 run hid_tools inspect_device --vid 0xVVVV --pid 0xPPPP
```

This shows raw reports your device sends.

## Example Projects

See `examples/` directory for complete firmware examples.
