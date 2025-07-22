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

#include "dummy_pose_broadcaster/dummy_pose_broadcaster.hpp"

#include <memory>
#include <string>

namespace dummy_pose_broadcaster
{

DummyPoseBroadcaster::DummyPoseBroadcaster()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DummyPoseBroadcaster::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::string>("sensor_name", "hid_input");
    auto_declare<std::string>("frame_id", "dummy_hid_frame");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during init stage with message: %s", e.what());

    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DummyPoseBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
DummyPoseBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  // Use ALL available state interfaces from the sensor
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn DummyPoseBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configuring DummyPoseBroadcaster with sensor: %s, frame: %s",
    sensor_name_.c_str(), frame_id_.c_str());

  // Initialize the pose publisher
  auto pose_pub = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("dummy_pose", 10);
  pose_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(pose_pub);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyPoseBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Log available state interfaces
  RCLCPP_INFO(
    get_node()->get_logger(),
    "DummyPoseBroadcaster has %zu state interfaces:", state_interfaces_.size());

  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "  [%zu] %s", i, state_interfaces_[i].get_name().c_str());
  }

  // Minimum check: we need at least position data
  if (state_interfaces_.size() < 4) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected at least 4 state interfaces (report_id, x, y, z), got %zu",
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "DummyPoseBroadcaster activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyPoseBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "DummyPoseBroadcaster deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DummyPoseBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Dynamically find interface indices by name
  // This way we don't hardcode the order or specific interface names
  auto find_interface = [this](const std::string & suffix) -> double {
      for (const auto & iface : state_interfaces_) {
        if (iface.get_name().find(suffix) != std::string::npos) {
          return iface.get_value();
        }
      }
      return 0.0; // Default if not found
    };

  // Read values by searching for interface name suffixes
  double x = find_interface("/x");
  double y = find_interface("/y");
  double z = find_interface("/z");
  double rx = find_interface("/rx");
  double ry = find_interface("/ry");
  double rz = find_interface("/rz");

  // Create and publish pose message
  if (pose_publisher_->trylock()) {
    pose_publisher_->msg_.header.stamp = time;
    pose_publisher_->msg_.header.frame_id = frame_id_;

    // Map HID input values to 6DoF pose
    // Position (x, y, z) scaled from uint8 (0-255) to meters (-1 to 1)
    pose_publisher_->msg_.pose.position.x = (x - 128.0) / 128.0;
    pose_publisher_->msg_.pose.position.y = (y - 128.0) / 128.0;
    pose_publisher_->msg_.pose.position.z = (z - 128.0) / 128.0;

    // Orientation quaternion from rx, ry, rz (Euler angles)
    double yaw = (rz - 128.0) / 128.0 * M_PI;    // Scale to -pi to pi
    double pitch = (ry - 128.0) / 128.0 * M_PI / 2;  // Scale to -pi/2 to pi/2
    double roll = (rx - 128.0) / 128.0 * M_PI / 2;   // Scale to -pi/2 to pi/2

    // Convert to quaternion (yaw, pitch, roll)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    pose_publisher_->msg_.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose_publisher_->msg_.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose_publisher_->msg_.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose_publisher_->msg_.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    pose_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace dummy_pose_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dummy_pose_broadcaster::DummyPoseBroadcaster,
  controller_interface::ControllerInterface)
