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

#ifndef SIGNAL_COMMAND_CONTROLLER__SIGNAL_COMMAND_CONTROLLER_HPP_
#define SIGNAL_COMMAND_CONTROLLER__SIGNAL_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "signal_command_controller/visibility_control.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace signal_command_controller
{

class SignalCommandController : public controller_interface::ControllerInterface
{
public:
  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  SignalCommandController();

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SIGNAL_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string gpio_name_;
  double update_interval_;  // Seconds between parameter updates

  // Current command values
  double amplitude_{1.0};
  double frequency_{1.0};
  double phase_{0.0};

  // Timing
  rclcpp::Time last_update_time_;

  // Publisher for current parameters
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> params_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    realtime_params_publisher_;
};

}  // namespace signal_command_controller

#endif  // SIGNAL_COMMAND_CONTROLLER__SIGNAL_COMMAND_CONTROLLER_HPP_
