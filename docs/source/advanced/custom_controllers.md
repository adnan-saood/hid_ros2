# Custom Controllers

Creating custom ros2_control controllers for HID devices.

## Overview

While HID ROS 2 provides generic controllers, you can create custom controllers for specialized behavior.

## Controller Basics

Controllers in ros2_control:
- Read from state interfaces
- Write to command interfaces
- Publish/subscribe ROS 2 topics
- Implement custom logic

## Creating a Controller

### 1. Package Setup

```bash
ros2 pkg create --build-type ament_cmake my_hid_controller \
  --dependencies controller_interface rclcpp
```

### 2. Controller Class

```cpp
#include "controller_interface/controller_interface.hpp"

class MyHidController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
};
```

### 3. Specify Interfaces

```cpp
controller_interface::InterfaceConfiguration
MyHidController::state_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {"hid_sensor/position_x", "hid_sensor/position_y"}
    };
}
```

### 4. Implement Update

```cpp
controller_interface::return_type MyHidController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    // Read state interfaces
    double x = state_interfaces_[0].get_value();
    double y = state_interfaces_[1].get_value();

    // Custom processing
    auto msg = std::make_shared<geometry_msgs::msg::Point>();
    msg->x = x;
    msg->y = y;

    // Publish
    publisher_->publish(*msg);

    return controller_interface::return_type::OK;
}
```

## Examples

See `signal_command_controller` for a complete bidirectional example.

## Best Practices

- Use lifecycle states properly
- Handle disconnections gracefully
- Validate interface availability
- Document interface requirements

For more details, see the [ros2_control documentation](https://control.ros.org).
