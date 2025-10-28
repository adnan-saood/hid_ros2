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

#ifndef HID_HARDWARE__HID_HARDWARE_HPP_
#define HID_HARDWARE__HID_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hid_hardware/hid_device.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hid_hardware
{
class HidHardware : public hardware_interface::SystemInterface
{
public:
  using CALLBACK_RETURN = hardware_interface::CallbackReturn;
  using HW_RETURN_TYPE = hardware_interface::return_type;

  CALLBACK_RETURN on_init(const hardware_interface::HardwareInfo & info) override;
  CALLBACK_RETURN on_configure(const rclcpp_lifecycle::State & previous_state) override; 
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CALLBACK_RETURN on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CALLBACK_RETURN on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  HW_RETURN_TYPE read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  HW_RETURN_TYPE write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  void parse_type_info();
  std::vector<std::string> split_string(const std::string& s, char delimiter);
  size_t get_type_size(const std::string& type);
  double bytes_to_value(const unsigned char* bytes, const std::string& type);
  void value_to_bytes(double value, const std::string& type, unsigned char* bytes);
  void detect_report_size();

  std::vector<double> state_data;
  std::vector<double> command_data;
  std::vector<uint8_t> input_buffer_;

  hid_device * hid_device_{nullptr};
  uint16_t vendor_id_{0};
  uint16_t product_id_{0};
  size_t input_report_size_{0};

  std::vector<std::string> state_types_;
  std::vector<std::string> command_types_;
  std::string state_types_str_;
  std::string command_types_str_;
};

}  // namespace hid_hardware

#endif  // HID_HARDWARE__HID_HARDWARE_HPP_
