# HID Generic Broadcaster

A **fully adaptive** ROS2 controller that automatically works with **any** HID device schema - mouse, gamepad, microcontroller, or custom device.

## Features

✅ **Zero Configuration** - Works with any YAML schema automatically
✅ **Auto-Discovery** - Finds all state interfaces at runtime
✅ **Rate Control** - Configurable publish frequency
✅ **Universal** - Mouse, gamepad, 6DOF sensor, custom devices - all work
✅ **Diagnostic** - Logs all discovered interfaces on startup

## How It Works

The controller:
1. Claims **ALL** available state interfaces from hardware
2. Discovers their names and indices at activation
3. Publishes all values as `Float64MultiArray` at specified rate
4. Works with **any** device - no code changes needed!

## Published Topic

**Topic:** `/hid_states`
**Type:** `std_msgs/msg/Float64MultiArray`

```yaml
layout:
  dim:
    - label: "interfaces"
      size: 7          # Number of interfaces
      stride: 7
data: [1.0, 128.0, 130.0, 125.0, 128.0, 128.0, 128.0]
       # [report_id, x, y, z, rx, ry, rz]
```

The array indices correspond to the order logged at activation.

## Parameters

### `sensor_name` (string, default: "hid_input")
The name of the HID sensor to read from.

### `publish_rate` (double, default: 100.0)
Publishing frequency in Hz.

## Usage

### In Generated Launch File (Automatic)

The generator automatically adds this controller to your launch file. Just:

```bash
ros2 launch dummy_bringup hid.launch.py
```

### Monitor Output

```bash
# See all interface values in real-time
ros2 topic echo /hid_states

# See at specific rate
ros2 topic hz /hid_states
```

### View Interface Mapping

Check the launch logs to see which array index maps to which interface:

```
[INFO] [hid_generic_broadcaster]: HidGenericBroadcaster discovered 7 state interfaces:
[INFO] [hid_generic_broadcaster]:   [0] hid_input/report_id
[INFO] [hid_generic_broadcaster]:   [1] hid_input/x
[INFO] [hid_generic_broadcaster]:   [2] hid_input/y
[INFO] [hid_generic_broadcaster]:   [3] hid_input/z
[INFO] [hid_generic_broadcaster]:   [4] hid_input/rx
[INFO] [hid_generic_broadcaster]:   [5] hid_input/ry
[INFO] [hid_generic_broadcaster]:   [6] hid_input/rz
```

## Examples

### Example 1: 6DOF Sensor

**YAML Schema:**
```yaml
fields:
  - name: "x"
    type: "uint8"
  - name: "y"
    type: "uint8"
  - name: "z"
    type: "uint8"
  - name: "rx"
    type: "uint8"
  - name: "ry"
    type: "uint8"
  - name: "rz"
    type: "uint8"
```

**Published Data:**
```
data: [1.0, 128.0, 130.0, 125.0, 128.0, 128.0, 128.0]
       # [report_id, x=128, y=130, z=125, rx=128, ry=128, rz=128]
```

### Example 2: Gaming Mouse

**YAML Schema:**
```yaml
vendor_id: "0x046d"
product_id: "0xc07e"
fields:
  - name: "dx"
    type: "int16"
  - name: "dy"
    type: "int16"
  - name: "wheel"
    type: "int8"
  - name: "buttons"
    type: "uint8"
    count: 2
```

**Published Data:**
```
data: [1.0, 5.0, -3.0, 0.0, 0.0, 1.0]
       # [report_id, dx=5, dy=-3, wheel=0, buttons_0=0, buttons_1=1]
```

### Example 3: Gamepad

**YAML Schema:**
```yaml
fields:
  - name: "left_stick_x"
    type: "int16"
  - name: "left_stick_y"
    type: "int16"
  - name: "right_stick_x"
    type: "int16"
  - name: "right_stick_y"
    type: "int16"
  - name: "triggers"
    type: "uint8"
    count: 2
  - name: "buttons"
    type: "uint16"
```

**Published Data:**
```
data: [1.0, 0.0, 0.0, 127.0, -50.0, 255.0, 128.0, 4096.0]
       # All interfaces automatically discovered and published!
```

### Example 4: Custom Sensor Array

**YAML Schema:**
```yaml
fields:
  - name: "temperature"
    type: "int16"
  - name: "humidity"
    type: "uint16"
  - name: "pressure"
    type: "uint32"
  - name: "accel"
    type: "int16"
    count: 3
  - name: "gyro"
    type: "int16"
    count: 3
```

**Published Data:**
```
data: [1.0, 2350.0, 6500.0, 101325.0, -120.0, 50.0, 980.0, 5.0, -2.0, 1.0]
       # [report_id, temp, humidity, pressure, accel_0-2, gyro_0-2]
```

## Integration with Your Code

### Python Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HidListener(Node):
    def __init__(self):
        super().__init__('hid_listener')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'hid_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Check launch logs for index mapping
        # Example for 6DOF: [report_id, x, y, z, rx, ry, rz]
        if len(msg.data) >= 7:
            x = msg.data[1]
            y = msg.data[2]
            z = msg.data[3]
            rx = msg.data[4]
            ry = msg.data[5]
            rz = msg.data[6]

            self.get_logger().info(f'Position: ({x}, {y}, {z}), Rotation: ({rx}, {ry}, {rz})')
```

### C++ Subscriber

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HidListener : public rclcpp::Node
{
public:
  HidListener() : Node("hid_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "hid_states", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 7) {
          double x = msg->data[1];
          double y = msg->data[2];
          // Process data...
        }
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};
```

## Advantages Over Custom Controllers

| Feature | Custom Controller | Generic Broadcaster |
|---------|------------------|-------------------|
| Works with any device | ❌ No | ✅ Yes |
| Requires code changes | ✅ Yes | ❌ No |
| Auto-adapts to schema | ❌ No | ✅ Yes |
| Rebuild needed | ✅ Yes | ❌ No |
| Interface discovery | Manual | ✅ Automatic |
| Diagnostic logging | ❌ Usually no | ✅ Yes |

## Workflow

```bash
# 1. Edit your YAML schema (any device, any fields)
vim dummy_bringup/schema/hid_device.yaml

# 2. Rebuild (only bringup package!)
colcon build --packages-select dummy_bringup

# 3. Launch
ros2 launch dummy_bringup hid.launch.py

# 4. Monitor output
ros2 topic echo /hid_states

# 5. Check logs for interface mapping
# Look for: "[INFO] [hid_generic_broadcaster]: discovered X state interfaces"
```

## When to Use

### Use Generic Broadcaster when:
- ✅ You want raw HID data
- ✅ Device schema changes frequently
- ✅ Multiple different devices
- ✅ Quick prototyping
- ✅ Data logging/recording
- ✅ Debugging

### Use Custom Controller when:
- ✅ Need specific message types (Pose, Twist, Joy, etc.)
- ✅ Complex data transformations
- ✅ Specific application logic
- ✅ RViz visualization (needs specific msg types)

## Combining Both

You can run **both** controllers simultaneously:
- `hid_generic_broadcaster` → Raw data as Float64MultiArray
- `dummy_pose_broadcaster` → Processed data as PoseStamped

Both read the same hardware interfaces!

## Troubleshooting

### No data published

Check that the controller is active:
```bash
ros2 control list_controllers
# Should show: hid_generic_broadcaster [active]
```

### Wrong number of elements

Check the activation logs to see how many interfaces were discovered:
```bash
# In launch output, look for:
[INFO] [hid_generic_broadcaster]: discovered 7 state interfaces
```

### Interface mapping unclear

Always check launch logs on startup - they show the exact index→name mapping.

## Summary

This controller is **the universal adapter** for any HID device. Change your device, change your schema, add fields, remove fields - it just works!

No code changes. No rebuilds. Just edit YAML and launch.
