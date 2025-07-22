// Copyright 2025 Adnan Saood
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//

#ifndef HID_GENERIC_BROADCASTER__HID_GENERIC_BROADCASTER_HPP_
#define HID_GENERIC_BROADCASTER__HID_GENERIC_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hid_generic_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hid_generic_broadcaster
{

class HidGenericBroadcaster : public controller_interface::ControllerInterface
{
public:
  HID_GENERIC_BROADCASTER_PUBLIC
  HidGenericBroadcaster();

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_GENERIC_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string sensor_name_;
  double publish_rate_;

  // Track last publish time for rate limiting
  rclcpp::Time last_publish_time_;

  // Interface metadata storage
  struct InterfaceInfo {
    std::string name;
    size_t index;
  };
  std::vector<InterfaceInfo> interface_info_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    realtime_state_publisher_;
};

}  // namespace hid_generic_broadcaster

#endif  // HID_GENERIC_BROADCASTER__HID_GENERIC_BROADCASTER_HPP_
