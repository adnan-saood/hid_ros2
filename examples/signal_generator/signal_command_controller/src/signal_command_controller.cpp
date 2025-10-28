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

#include "signal_command_controller/signal_command_controller.hpp"

#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace signal_command_controller
{

SignalCommandController::SignalCommandController()
  : controller_interface::ControllerInterface(),
    update_interval_(10.0)
{}

controller_interface::CallbackReturn SignalCommandController::on_init()
{
  try
  {
    auto_declare<std::string>("gpio_name", "signal_params");
    auto_declare<double>("update_interval", 10.0);  // seconds
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during init stage with message: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SignalCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::InterfaceConfiguration
SignalCommandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn SignalCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  gpio_name_ = get_node()->get_parameter("gpio_name").as_string();
  update_interval_ = get_node()->get_parameter("update_interval").as_double();

  if (update_interval_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Update interval must be positive, got: %.2f", update_interval_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configuring SignalCommandController: gpio=%s, update_interval=%.1fs",
    gpio_name_.c_str(), update_interval_);

  // Initialize publisher for current parameters
  params_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/current_params", 10);
  realtime_params_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      params_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SignalCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the expected command interfaces
  if (command_interfaces_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 3 command interfaces, got %zu", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "SignalCommandController activated with command interfaces:");

  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "  [%zu] %s", i, command_interfaces_[i].get_name().c_str());
  }

  // Initialize timing
  last_update_time_ = get_node()->now();

  // Set initial values
  amplitude_ = 1.0;
  frequency_ = 1.0;
  phase_ = 0.0;

  // Write initial values to command interfaces
  command_interfaces_[0].set_value(amplitude_);
  command_interfaces_[1].set_value(frequency_);
  command_interfaces_[2].set_value(phase_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Initial parameters: amplitude=%.2f, frequency=%.2f, phase=%.2f",
    amplitude_, frequency_, phase_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SignalCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "SignalCommandController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SignalCommandController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Check if it's time to update parameters
  const double seconds_since_last_update = (time - last_update_time_).seconds();

  if (seconds_since_last_update >= update_interval_) {
    last_update_time_ = time;

    // Generate new parameters - cycling through different values
    static int cycle = 0;
    cycle++;

    // Amplitude: cycle between 0.5, 1.0, 1.5, 2.0
    amplitude_ = 0.5 + (cycle % 4) * 0.5;

    // Frequency: cycle between 0.5, 1.0, 2.0, 3.0
    const double freqs[] = {0.5, 1.0, 2.0, 3.0};
    frequency_ = freqs[cycle % 4];

    // Phase: cycle between 0, π/4, π/2, π
    const double phases[] = {0.0, M_PI/4, M_PI/2, M_PI};
    phase_ = phases[cycle % 4];

    // Write new values to command interfaces
    command_interfaces_[0].set_value(amplitude_);
    command_interfaces_[1].set_value(frequency_);
    command_interfaces_[2].set_value(phase_);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Updated parameters (cycle %d): amplitude=%.2f, frequency=%.2f, phase=%.2f",
      cycle, amplitude_, frequency_, phase_);
  }

  // Publish current parameters
  if (realtime_params_publisher_->trylock()) {
    realtime_params_publisher_->msg_.data.resize(3);
    realtime_params_publisher_->msg_.data[0] = amplitude_;
    realtime_params_publisher_->msg_.data[1] = frequency_;
    realtime_params_publisher_->msg_.data[2] = phase_;
    realtime_params_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace signal_command_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  signal_command_controller::SignalCommandController,
  controller_interface::ControllerInterface)
