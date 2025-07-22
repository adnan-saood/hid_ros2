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

#include "hid_generic_broadcaster/hid_generic_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>

namespace hid_generic_broadcaster
{

HidGenericBroadcaster::HidGenericBroadcaster()
  : controller_interface::ControllerInterface(),
    publish_rate_(100.0)
{}

controller_interface::CallbackReturn HidGenericBroadcaster::on_init()
{
  try
  {
    // Declare parameters with defaults
    auto_declare<std::string>("sensor_name", "hid_input");
    auto_declare<double>("publish_rate", 100.0);  // Hz
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
HidGenericBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
HidGenericBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  // Automatically claim ALL available state interfaces
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn HidGenericBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Publish rate must be positive, got: %.2f", publish_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configuring HidGenericBroadcaster with sensor: %s, publish_rate: %.1f Hz",
    sensor_name_.c_str(), publish_rate_);

  // Initialize the state publisher
  state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "hid_states", 10);
  realtime_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      state_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HidGenericBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear previous interface info
  interface_info_.clear();

  // Store metadata about all discovered interfaces
  RCLCPP_INFO(
    get_node()->get_logger(),
    "HidGenericBroadcaster discovered %zu state interfaces:",
    state_interfaces_.size());

  // Build parameter arrays for interface names and indices
  std::vector<std::string> interface_names;

  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto& iface = state_interfaces_[i];
    interface_info_.push_back({iface.get_name(), i});
    interface_names.push_back(iface.get_name());

    RCLCPP_INFO(
      get_node()->get_logger(),
      "  [%zu] %s", i, iface.get_name().c_str());
  }

  if (state_interfaces_.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "No state interfaces found! Controller will publish empty arrays.");
  }

  // Declare read-only parameter with interface names
  try {
    get_node()->declare_parameter("interface_names", interface_names);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Interface mapping published as parameter 'interface_names'");
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Failed to declare interface_names parameter: %s", e.what());
  }

  // Pre-allocate the message arrays
  if (realtime_state_publisher_->trylock()) {
    realtime_state_publisher_->msg_.data.resize(state_interfaces_.size());
    realtime_state_publisher_->msg_.layout.dim.resize(1);
    realtime_state_publisher_->msg_.layout.dim[0].label = "interfaces";
    realtime_state_publisher_->msg_.layout.dim[0].size = state_interfaces_.size();
    realtime_state_publisher_->msg_.layout.dim[0].stride = state_interfaces_.size();
    realtime_state_publisher_->unlock();
  }

  // Initialize last publish time
  last_publish_time_ = get_node()->now();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "HidGenericBroadcaster activated successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HidGenericBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "HidGenericBroadcaster deactivated");
  interface_info_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HidGenericBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Check if it's time to publish based on publish_rate
  const double seconds_since_last_publish = (time - last_publish_time_).seconds();
  const double publish_period = 1.0 / publish_rate_;

  if (seconds_since_last_publish < publish_period) {
    return controller_interface::return_type::OK;
  }

  // Try to publish
  if (realtime_state_publisher_->trylock()) {
    // Update timestamp
    last_publish_time_ = time;

    // Read all state interface values into the message
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      realtime_state_publisher_->msg_.data[i] = state_interfaces_[i].get_value();
    }

    realtime_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace hid_generic_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hid_generic_broadcaster::HidGenericBroadcaster,
  controller_interface::ControllerInterface)
