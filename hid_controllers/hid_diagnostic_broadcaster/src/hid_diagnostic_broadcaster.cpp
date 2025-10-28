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

#include "hid_diagnostic_broadcaster/hid_diagnostic_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace hid_diagnostic_broadcaster
{

HidDiagnosticBroadcaster::HidDiagnosticBroadcaster()
  : controller_interface::ControllerInterface(),
    publish_rate_(1.0)
{}

controller_interface::CallbackReturn HidDiagnosticBroadcaster::on_init()
{
  try
  {
    // Declare parameters with defaults
    auto_declare<std::string>("sensor_name", "hid_input");
    auto_declare<double>("publish_rate", 1.0);  // Hz - diagnostics don't need high rate
    auto_declare<std::string>("device_name", "Unknown HID Device");
    auto_declare<std::string>("vendor_id", "0x0000");
    auto_declare<std::string>("product_id", "0x0000");
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
HidDiagnosticBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
HidDiagnosticBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  // Automatically claim ALL available state interfaces
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn HidDiagnosticBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  device_name_ = get_node()->get_parameter("device_name").as_string();
  vendor_id_ = get_node()->get_parameter("vendor_id").as_string();
  product_id_ = get_node()->get_parameter("product_id").as_string();

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Publish rate must be positive, got: %.2f", publish_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configuring HidDiagnosticBroadcaster for device: %s (VID=%s, PID=%s), publish_rate: %.1f Hz",
    device_name_.c_str(), vendor_id_.c_str(), product_id_.c_str(), publish_rate_);

  // Initialize the diagnostic publisher
  diagnostic_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);
  realtime_diagnostic_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      diagnostic_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HidDiagnosticBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Store interface names
  interface_names_.clear();
  last_values_.clear();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "HidDiagnosticBroadcaster discovered %zu state interfaces",
    state_interfaces_.size());

  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto& iface = state_interfaces_[i];
    interface_names_.push_back(iface.get_name());
    last_values_.push_back(0.0);
  }

  // Initialize timing
  last_publish_time_ = get_node()->now();
  last_update_time_ = get_node()->now();
  last_change_time_ = get_node()->now();
  stats_start_time_ = get_node()->now();
  read_count_ = 0;
  stats_read_count_ = 0;
  last_read_rate_ = 0.0;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "HidDiagnosticBroadcaster activated - publishing to /diagnostics");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HidDiagnosticBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "HidDiagnosticBroadcaster deactivated");
  interface_names_.clear();
  last_values_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HidDiagnosticBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Track update rate
  read_count_++;
  stats_read_count_++;

  // Check for data changes
  bool data_changed = false;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    double current_value = state_interfaces_[i].get_value();
    if (std::abs(current_value - last_values_[i]) > 1e-9) {
      last_values_[i] = current_value;
      data_changed = true;
    }
  }

  if (data_changed) {
    last_change_time_ = time;
  }

  // Calculate read rate every second
  const double stats_period = (time - stats_start_time_).seconds();
  if (stats_period >= 1.0) {
    last_read_rate_ = stats_read_count_ / stats_period;
    stats_start_time_ = time;
    stats_read_count_ = 0;
  }

  // Check if it's time to publish diagnostics
  const double seconds_since_last_publish = (time - last_publish_time_).seconds();
  const double publish_period = 1.0 / publish_rate_;

  if (seconds_since_last_publish < publish_period) {
    return controller_interface::return_type::OK;
  }

  // Try to publish diagnostics
  if (realtime_diagnostic_publisher_->trylock()) {
    last_publish_time_ = time;

    auto & msg = realtime_diagnostic_publisher_->msg_;
    msg.header.stamp = time;
    msg.status.clear();

    // Create diagnostic status
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_node()->get_name();
    status.hardware_id = device_name_ + " (" + vendor_id_ + ":" + product_id_ + ")";

    // Determine status level
    const double time_since_change = (time - last_change_time_).seconds();
    if (state_interfaces_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "No state interfaces available";
    } else if (time_since_change > 1.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Data not changing (device may be frozen)";
    } else if (last_read_rate_ < 10.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Low read rate";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "HID device operating normally";
    }

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "Device Name";
    kv.value = device_name_;
    status.values.push_back(kv);

    kv.key = "Vendor ID";
    kv.value = vendor_id_;
    status.values.push_back(kv);

    kv.key = "Product ID";
    kv.value = product_id_;
    status.values.push_back(kv);

    kv.key = "Read Rate (Hz)";
    kv.value = std::to_string(last_read_rate_);
    status.values.push_back(kv);

    kv.key = "Total Reads";
    kv.value = std::to_string(read_count_);
    status.values.push_back(kv);

    kv.key = "Interface Count";
    kv.value = std::to_string(state_interfaces_.size());
    status.values.push_back(kv);

    kv.key = "Time Since Last Change (s)";
    kv.value = std::to_string(time_since_change);
    status.values.push_back(kv);

    // Add current interface values
    for (size_t i = 0; i < state_interfaces_.size() && i < 10; ++i) {
      kv.key = interface_names_[i];
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << state_interfaces_[i].get_value();
      kv.value = oss.str();
      status.values.push_back(kv);
    }

    if (state_interfaces_.size() > 10) {
      kv.key = "... (additional interfaces)";
      kv.value = std::to_string(state_interfaces_.size() - 10) + " more";
      status.values.push_back(kv);
    }

    msg.status.push_back(status);
    realtime_diagnostic_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace hid_diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hid_diagnostic_broadcaster::HidDiagnosticBroadcaster,
  controller_interface::ControllerInterface)
