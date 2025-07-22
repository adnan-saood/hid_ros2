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

#ifndef DUMMY_POSE_BROADCASTER__DUMMY_POSE_BROADCASTER_HPP_
#define DUMMY_POSE_BROADCASTER__DUMMY_POSE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "dummy_pose_broadcaster/visibility_control.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace dummy_pose_broadcaster
{

class DummyPoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  DUMMY_POSE_BROADCASTER_PUBLIC
  DummyPoseBroadcaster();

  DUMMY_POSE_BROADCASTER_PUBLIC
  virtual ~DummyPoseBroadcaster() = default;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DUMMY_POSE_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string sensor_name_;
  std::string frame_id_;

  std::vector<std::string> state_interface_names_;

  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
  pose_publisher_;
};

}  // namespace dummy_pose_broadcaster

#endif  // DUMMY_POSE_BROADCASTER__DUMMY_POSE_BROADCASTER_HPP_
