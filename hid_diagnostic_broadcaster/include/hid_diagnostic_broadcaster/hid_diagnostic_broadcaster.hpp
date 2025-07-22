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

#ifndef HID_DIAGNOSTIC_BROADCASTER__HID_DIAGNOSTIC_BROADCASTER_HPP_
#define HID_DIAGNOSTIC_BROADCASTER__HID_DIAGNOSTIC_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hid_diagnostic_broadcaster/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace hid_diagnostic_broadcaster
{

class HidDiagnosticBroadcaster : public controller_interface::ControllerInterface
{
public:
  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  HidDiagnosticBroadcaster();

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HID_DIAGNOSTIC_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string sensor_name_;
  double publish_rate_;
  std::string device_name_;
  std::string vendor_id_;
  std::string product_id_;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostic_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    realtime_diagnostic_publisher_;

  // Timing
  rclcpp::Time last_publish_time_;
  rclcpp::Time last_update_time_;

  // Statistics
  size_t read_count_{0};
  double last_read_rate_{0.0};
  rclcpp::Time stats_start_time_;
  size_t stats_read_count_{0};

  // Interface tracking
  std::vector<std::string> interface_names_;
  std::vector<double> last_values_;
  rclcpp::Time last_change_time_;
};

}  // namespace hid_diagnostic_broadcaster

#endif  // HID_DIAGNOSTIC_BROADCASTER__HID_DIAGNOSTIC_BROADCASTER_HPP_
