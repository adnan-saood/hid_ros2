# Understanding HID Report Descriptors

A comprehensive, **interactive** guide to USB HID report descriptors - what they are, how they work, and how to interpret every byte.

```{contents} Table of Contents
:depth: 3
:local:
```

## What is a HID Report Descriptor?

A HID report descriptor is a binary data structure that tells the USB host (your computer) exactly how to interpret the data sent by your HID device. Think of it as a "data dictionary" that describes:

- **What data** the device sends (inputs) and receives (outputs)
- **How much data** (size in bits/bytes)
- **What format** the data is in (signed/unsigned, range, units)
- **What it represents** (buttons, axes, sensors, LEDs)

Without a report descriptor, the raw bytes from your device would be meaningless. The descriptor gives them structure and meaning.

## Report Structure Overview

### Reports vs. Descriptors

**Report Descriptor** (sent once at connection):
```
"I will send you 8 bytes. The first 4 bytes are a float representing
temperature, the next 2 bytes are a signed 16-bit X position, and the
last 2 bytes are a signed 16-bit Y position."
```

**Report Data** (sent repeatedly during operation):
```
[0x42, 0x28, 0x00, 0x00, 0xFF, 0x64, 0x00, 0xC8]
 └─────────┬─────────┘  └──┬──┘  └──┬──┘
      Temperature      X=100    Y=200
      (25.5°C)
```

The descriptor is the **schema**, the report is the **data**.

## Interactive Example: Temperature Sensor

Let's build a complete descriptor for a simple temperature sensor. **Hover over each byte** to see what it means!

```{raw} html
<div class="hid-descriptor-container">
  <div class="hid-descriptor-title">🌡️ Temperature Sensor Descriptor</div>
  <div class="hid-bytes-container">
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0xA1</span>
      <span class="hid-byte-label">COLLECT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Collection (Application)</span>
        <span class="hid-byte-tooltip-desc">Starts a new HID application collection. All items inside belong to this device.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">APP</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Application Type</span>
        <span class="hid-byte-tooltip-desc">This collection represents a complete application (as opposed to physical, logical, etc.)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x05</span>
      <span class="hid-byte-label">USAGE PG</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage Page (Global)</span>
        <span class="hid-byte-tooltip-desc">Sets the category for all following items. Affects all subsequent fields until changed.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">GEN DESK</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Generic Desktop</span>
        <span class="hid-byte-tooltip-desc">Usage Page 0x01 = Generic Desktop Controls (mouse, keyboard, joystick, etc.)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-local">
      <span class="hid-byte-value">0x09</span>
      <span class="hid-byte-label">USAGE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage (Local)</span>
        <span class="hid-byte-tooltip-desc">Defines what this specific field represents. Only affects the next Main item.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x00</span>
      <span class="hid-byte-label">CUSTOM</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Undefined/Custom</span>
        <span class="hid-byte-tooltip-desc">Usage 0x00 = Undefined. We're using a custom/vendor-defined sensor.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x16</span>
      <span class="hid-byte-label">LOG MIN</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Logical Minimum (2 bytes)</span>
        <span class="hid-byte-tooltip-desc">Sets the minimum value for data fields. 0x16 means 2 bytes of data follow.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x00</span>
      <span class="hid-byte-label">LOW</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Min Value Low Byte</span>
        <span class="hid-byte-tooltip-desc">Low byte of minimum value: 0x8000 = -32768 (little-endian)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x80</span>
      <span class="hid-byte-label">HIGH</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Min Value High Byte</span>
        <span class="hid-byte-tooltip-desc">High byte of minimum value: 0x8000 = -32768 (signed 16-bit min)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x26</span>
      <span class="hid-byte-label">LOG MAX</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Logical Maximum (2 bytes)</span>
        <span class="hid-byte-tooltip-desc">Sets the maximum value for data fields. 0x26 means 2 bytes of data follow.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0xFF</span>
      <span class="hid-byte-label">LOW</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Max Value Low Byte</span>
        <span class="hid-byte-tooltip-desc">Low byte of maximum value: 0x7FFF = 32767 (little-endian)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x7F</span>
      <span class="hid-byte-label">HIGH</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Max Value High Byte</span>
        <span class="hid-byte-tooltip-desc">High byte of maximum value: 0x7FFF = 32767 (signed 16-bit max)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x75</span>
      <span class="hid-byte-label">RPT SIZE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report Size</span>
        <span class="hid-byte-tooltip-desc">Number of BITS per field. This affects all following fields.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x10</span>
      <span class="hid-byte-label">16 bits</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">16 Bits</span>
        <span class="hid-byte-tooltip-desc">0x10 = 16 decimal. Each field will be 16 bits = 2 bytes.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x95</span>
      <span class="hid-byte-label">RPT CNT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report Count</span>
        <span class="hid-byte-tooltip-desc">Number of fields. Total data = Report Size × Report Count.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">1 field</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">1 Field</span>
        <span class="hid-byte-tooltip-desc">Send 1 field of 16 bits = 2 bytes total per report.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0x81</span>
      <span class="hid-byte-label">INPUT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Input (Main Item)</span>
        <span class="hid-byte-tooltip-desc">Declares an input field (device → host). Uses all previously set Global and Local items.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x02</span>
      <span class="hid-byte-label">VAR</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Data, Variable, Absolute</span>
        <span class="hid-byte-tooltip-desc">0x02 = 0b00000010. Bit 0=Data (not const), Bit 1=Variable (not array), Bit 2=Absolute (not relative)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0xC0</span>
      <span class="hid-byte-label">END</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">End Collection</span>
        <span class="hid-byte-tooltip-desc">Closes the application collection started with 0xA1.</span>
      </div>
    </div>
  </div>

  <div class="hid-legend">
    <div class="hid-legend-item">
      <div class="hid-legend-box main"></div>
      <span>Main Item</span>
    </div>
    <div class="hid-legend-item">
      <div class="hid-legend-box global"></div>
      <span>Global Item</span>
    </div>
    <div class="hid-legend-item">
      <div class="hid-legend-box local"></div>
      <span>Local Item</span>
    </div>
    <div class="hid-legend-item">
      <div class="hid-legend-box data"></div>
      <span>Data Byte</span>
    </div>
  </div>
</div>

<div class="descriptor-explanation">
  <h4>📊 What This Descriptor Creates</h4>
  <p>The above descriptor (15 bytes total) tells the USB host:</p>
  <ul>
    <li><strong>Data type:</strong> Signed 16-bit integer (range -32768 to 32767)</li>
    <li><strong>Report structure:</strong> 1 field × 16 bits = 2 bytes per report</li>
    <li><strong>Direction:</strong> Input (device sends to host)</li>
  </ul>

  <p>Your firmware structure would look like:</p>
</div>
```

```c
struct temperature_report {
    int16_t temperature;  // -32768 to 32767
} __attribute__((packed));
// Total: 2 bytes
```

## Interactive Example: 3-Axis Accelerometer

Now let's look at a more complex example with multiple fields. **Hover over each byte** to understand the structure!

```{raw} html
<div class="hid-descriptor-container">
  <div class="hid-descriptor-title">🎯 3-Axis Accelerometer Descriptor</div>
  <div class="hid-bytes-container">
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0xA1</span>
      <span class="hid-byte-label">COLLECT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Collection (Application)</span>
        <span class="hid-byte-tooltip-desc">Start of application collection</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">APP</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Application Type</span>
        <span class="hid-byte-tooltip-desc">Collection type = Application</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x85</span>
      <span class="hid-byte-label">RPT ID</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report ID</span>
        <span class="hid-byte-tooltip-desc">Assigns an ID to this report type. First byte of every report will be this ID.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">ID=1</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report ID 1</span>
        <span class="hid-byte-tooltip-desc">This report will have ID = 1. Useful when device has multiple report types.</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x05</span>
      <span class="hid-byte-label">USAGE PG</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage Page</span>
        <span class="hid-byte-tooltip-desc">Sets data category</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x01</span>
      <span class="hid-byte-label">GEN DESK</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Generic Desktop</span>
        <span class="hid-byte-tooltip-desc">Category: Generic Desktop Controls</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-local">
      <span class="hid-byte-value">0x09</span>
      <span class="hid-byte-label">USAGE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage (X axis)</span>
        <span class="hid-byte-tooltip-desc">First field will be X axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x30</span>
      <span class="hid-byte-label">X</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">X Axis</span>
        <span class="hid-byte-tooltip-desc">Usage 0x30 in Generic Desktop = X axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-local">
      <span class="hid-byte-value">0x09</span>
      <span class="hid-byte-label">USAGE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage (Y axis)</span>
        <span class="hid-byte-tooltip-desc">Second field will be Y axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x31</span>
      <span class="hid-byte-label">Y</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Y Axis</span>
        <span class="hid-byte-tooltip-desc">Usage 0x31 in Generic Desktop = Y axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-local">
      <span class="hid-byte-value">0x09</span>
      <span class="hid-byte-label">USAGE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Usage (Z axis)</span>
        <span class="hid-byte-tooltip-desc">Third field will be Z axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x32</span>
      <span class="hid-byte-label">Z</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Z Axis</span>
        <span class="hid-byte-tooltip-desc">Usage 0x32 in Generic Desktop = Z axis</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x16</span>
      <span class="hid-byte-label">LOG MIN</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Logical Minimum</span>
        <span class="hid-byte-tooltip-desc">Minimum value (2 bytes follow)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x00</span>
      <span class="hid-byte-label">LOW</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">-32768 Low Byte</span>
        <span class="hid-byte-tooltip-desc">0x8000 little-endian = -32768</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x80</span>
      <span class="hid-byte-label">HIGH</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">-32768 High Byte</span>
        <span class="hid-byte-tooltip-desc">Min = -32768</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x26</span>
      <span class="hid-byte-label">LOG MAX</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Logical Maximum</span>
        <span class="hid-byte-tooltip-desc">Maximum value (2 bytes follow)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0xFF</span>
      <span class="hid-byte-label">LOW</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">32767 Low Byte</span>
        <span class="hid-byte-tooltip-desc">0x7FFF little-endian = 32767</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x7F</span>
      <span class="hid-byte-label">HIGH</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">32767 High Byte</span>
        <span class="hid-byte-tooltip-desc">Max = 32767</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x75</span>
      <span class="hid-byte-label">RPT SIZE</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report Size</span>
        <span class="hid-byte-tooltip-desc">Bits per field</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x10</span>
      <span class="hid-byte-label">16 bits</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">16 Bits Per Field</span>
        <span class="hid-byte-tooltip-desc">Each axis = 16 bits = 2 bytes</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-global">
      <span class="hid-byte-value">0x95</span>
      <span class="hid-byte-label">RPT CNT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Report Count</span>
        <span class="hid-byte-tooltip-desc">Number of fields</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x03</span>
      <span class="hid-byte-label">3 fields</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">3 Fields</span>
        <span class="hid-byte-tooltip-desc">3 axes × 16 bits = 48 bits = 6 bytes total</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0x81</span>
      <span class="hid-byte-label">INPUT</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Input (Main)</span>
        <span class="hid-byte-tooltip-desc">Declares input fields using all previous settings</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-data">
      <span class="hid-byte-value">0x02</span>
      <span class="hid-byte-label">VAR</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">Variable, Absolute</span>
        <span class="hid-byte-tooltip-desc">Data is variable and absolute (not relative)</span>
      </div>
    </div>
    <div class="hid-byte hid-byte-main">
      <span class="hid-byte-value">0xC0</span>
      <span class="hid-byte-label">END</span>
      <div class="hid-byte-tooltip">
        <span class="hid-byte-tooltip-title">End Collection</span>
        <span class="hid-byte-tooltip-desc">Closes the application collection</span>
      </div>
    </div>
  </div>
</div>

<div class="report-structure">
  <div class="report-structure-title">📦 Generated Report Structure</div>
  <div class="report-field">
    <span class="report-field-name">report_id</span>
    <span class="report-field-type">uint8_t (1 byte)</span>
  </div>
  <div class="report-field">
    <span class="report-field-name">x</span>
    <span class="report-field-type">int16_t (2 bytes)</span>
  </div>
  <div class="report-field">
    <span class="report-field-name">y</span>
    <span class="report-field-type">int16_t (2 bytes)</span>
  </div>
  <div class="report-field">
    <span class="report-field-name">z</span>
    <span class="report-field-type">int16_t (2 bytes)</span>
  </div>
  <div style="margin-top: 1em; padding: 12px; background: #edf2f7; border-radius: 4px; font-weight: bold; text-align: center;">
    Total Report Size: 7 bytes
  </div>
</div>
```

```c
struct accelerometer_report {
    uint8_t report_id;   // Always 0x01
    int16_t x;           // -32768 to 32767
    int16_t y;           // -32768 to 32767
    int16_t z;           // -32768 to 32767
} __attribute__((packed));
```

## Understanding Item Types

HID descriptors are built from three types of items, each serving a different purpose:

### Main Items (Red)

**Define inputs, outputs, and structure**

- `0xA1` - Collection (start grouping)
- `0xC0` - End Collection (end grouping)
- `0x81` - Input (device → host data)
- `0x91` - Output (host → device data)
- `0xB1` - Feature (bidirectional config)

Main items **consume** the Global and Local items that came before them.

### Global Items (Blue)

**Affect all following items until changed**

- `0x05` - Usage Page (data category)
- `0x15/0x16/0x17` - Logical Minimum (1/2/4 bytes)
- `0x25/0x26/0x27` - Logical Maximum (1/2/4 bytes)
- `0x75` - Report Size (bits per field)
- `0x95` - Report Count (number of fields)
- `0x85` - Report ID (report identifier)

Global items remain active until explicitly changed.

### Local Items (Green)

**Affect only the next Main item**

- `0x09` - Usage (what the field represents)
- `0x19` - Usage Minimum (range start)
- `0x29` - Usage Maximum (range end)

Local items are reset after each Main item.

## Byte-Level Item Format

Every HID item follows this structure:

```
┌──────────┬─────────────────┐
│ Prefix   │  Data (0-4      │
│ Byte     │  bytes)         │
└──────────┴─────────────────┘
```

### Prefix Byte Breakdown

```
       0x95
        │
    ┌───┴───┐
    1001 0101
    │  │ │  │
    │  │ │  └──> Size bits [1:0] = 01 (1 byte of data)
    │  │ └─────> Type bits [3:2] = 10 (Global item)
    │  └───────> Reserved
    └──────────> Tag [7:4] = 1001 (Report Count)
```

**Size field:**
- `00` = 0 bytes
- `01` = 1 byte
- `10` = 2 bytes
- `11` = 4 bytes

**Type field:**
- `00` = Main
- `01` = Global
- `10` = Local
- `11` = Reserved

## Report Size vs Report Count

This is the most important concept to understand:

### Report Size

**Number of BITS per individual field**

```
0x75, 0x10  →  16 bits per field
0x75, 0x08  →  8 bits per field
0x75, 0x01  →  1 bit per field (for buttons)
```

### Report Count

**Number of fields**

```
0x95, 0x03  →  3 fields
0x95, 0x01  →  1 field
0x95, 0x08  →  8 fields (e.g., 8 buttons)
```

### The Math

```
Total data bits = Report Size × Report Count

Example: 3 axes, 16 bits each
  0x75, 0x10  (Report Size = 16 bits)
  0x95, 0x03  (Report Count = 3)
  Total = 16 × 3 = 48 bits = 6 bytes
```

## Input vs Output Reports

### Input Reports (Device → Host)

```c
0x81, 0x02  // Input (Data, Variable, Absolute)
```

**Direction:** Device sends to computer
**Use cases:** Sensor data, button states, joystick position
**Flow:** `Firmware → USB → Host → ROS 2 Topic`

### Output Reports (Host → Device)

```c
0x91, 0x02  // Output (Data, Variable, Absolute)
```

**Direction:** Computer sends to device
**Use cases:** LED control, motor commands, haptic feedback
**Flow:** `ROS 2 Topic → Host → USB → Firmware`

### Bidirectional Example

```{raw} html
<div class="binary-breakdown">
  <div class="binary-breakdown-title">Device with Sensor Input + LED Output</div>
  <div class="binary-explanation">
    <strong>Input Report ID 1:</strong> Temperature sensor (2 bytes)<br>
    <strong>Output Report ID 2:</strong> LED brightness (1 byte)
  </div>
</div>
```

```c
const uint8_t descriptor[] = {
    0xA1, 0x01,          // Collection (Application)

    // INPUT: Temperature
    0x85, 0x01,          // Report ID (1)
    0x05, 0x01,          // Usage Page (Generic Desktop)
    0x15, 0x00,          // Logical Min (0)
    0x26, 0xFF, 0x00,    // Logical Max (255)
    0x75, 0x08,          // Report Size (8 bits)
    0x95, 0x01,          // Report Count (1)
    0x81, 0x02,          // Input

    // OUTPUT: LED
    0x85, 0x02,          // Report ID (2)
    0x05, 0x08,          // Usage Page (LEDs)
    0x15, 0x00,          // Logical Min (0)
    0x25, 0x64,          // Logical Max (100)
    0x75, 0x08,          // Report Size (8 bits)
    0x95, 0x01,          // Report Count (1)
    0x91, 0x02,          // Output

    0xC0                 // End Collection
};
```

## Main Item Flags

The byte following Input/Output/Feature contains important flags:

```{raw} html
<div class="binary-breakdown">
  <div class="binary-breakdown-title">0x02 = Data, Variable, Absolute</div>
  <div class="binary-bits">0 0 0 0 0 0 1 0</div>
  <div class="binary-explanation">
    <strong>Bit 0 (0):</strong> Data (not Constant)<br>
    <strong>Bit 1 (1):</strong> Variable (not Array)<br>
    <strong>Bit 2 (0):</strong> Absolute (not Relative)<br>
    <strong>Bits 3-7:</strong> Various other flags
  </div>
</div>
```

**Common values:**
- `0x01` - Constant (padding)
- `0x02` - Data, Variable, Absolute (sensors, values)
- `0x06` - Data, Variable, Relative (mouse movement)

## Usage Pages Reference

| Code | Name | Examples |
|------|------|----------|
| `0x01` | Generic Desktop | X, Y, Z, Rx, Ry, Rz, Wheel |
| `0x02` | Simulation | Steering, Throttle, Brake |
| `0x06` | Generic Device | Battery, Wireless Signal |
| `0x07` | Keyboard | Key codes, modifiers |
| `0x08` | LEDs | Num Lock, Caps Lock, indicators |
| `0x09` | Button | Button 1-65535 |
| `0x0C` | Consumer | Volume, Mute, Play/Pause |
| `0x20` | Sensors | Accelerometer, Gyro, Temperature |

## Common Mistakes

### ❌ Wrong: Report Size in bytes

```c
0x75, 0x02,  // Report Size (2) ← This means 2 BITS, not bytes!
```

### ✅ Correct: Report Size in bits

```c
0x75, 0x10,  // Report Size (16 bits) = 2 bytes
```

### ❌ Wrong: Forgetting padding

```c
// 3 buttons = 3 bits (missing 5 bits of padding!)
0x75, 0x01,  // Report Size (1 bit)
0x95, 0x03,  // Report Count (3)
0x81, 0x02,  // Input
```

### ✅ Correct: Add padding to byte boundary

```c
// 3 buttons + 5 padding bits = 1 byte
0x75, 0x01,  // Report Size (1 bit)
0x95, 0x03,  // Report Count (3 buttons)
0x81, 0x02,  // Input
0x95, 0x05,  // Report Count (5 padding bits)
0x81, 0x01,  // Input (Constant) ← PADDING
```

## Quick Reference

### Building a Descriptor

1. Start collection: `0xA1, 0x01`
2. Set Report ID (optional): `0x85, <id>`
3. Set Usage Page: `0x05, <page>`
4. For each field:
   - Set Usage: `0x09, <usage>`
   - Set Min/Max: `0x15/0x25` or `0x16/0x26`
   - Set Report Size: `0x75, <bits>`
   - Set Report Count: `0x95, <count>`
   - Add Input/Output: `0x81/0x91, 0x02`
5. Close collection: `0xC0`

### Type Quick Reference

| Data Type | Report Size | Min | Max |
|-----------|-------------|-----|-----|
| `uint8` | `0x75, 0x08` | `0x15, 0x00` | `0x25, 0xFF` |
| `int8` | `0x75, 0x08` | `0x15, 0x80` | `0x25, 0x7F` |
| `uint16` | `0x75, 0x10` | `0x16, 0x00, 0x00` | `0x26, 0xFF, 0xFF` |
| `int16` | `0x75, 0x10` | `0x16, 0x00, 0x80` | `0x26, 0xFF, 0x7F` |
| `uint32` | `0x75, 0x20` | `0x17, ...` | `0x27, ...` |
| `float32` | `0x75, 0x20` | Use physical range | Use physical range |

## See Also

- [Schema Reference](../schema_reference.md) - YAML to descriptor mapping
- [Type System](type_system.md) - Type conversions and limitations
- [Firmware Integration](firmware_integration.md) - Using descriptors in firmware
- [Debugging Guide](debugging.md) - Troubleshooting descriptor issues
