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

#include "hid_hardware/hid_hardware.hpp"

#include <hidapi/hidapi.h>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hid_hardware
{

  hardware_interface::CallbackReturn HidHardware::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get vendor_id and product_id from parameters
    vendor_id_ = static_cast<uint16_t>
    (std::stoul(info_.hardware_parameters.at("vendor_id"), nullptr, 16));

    product_id_ = static_cast<uint16_t>
    (std::stoul(info_.hardware_parameters.at("product_id"), nullptr, 16));

    RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                "Attempting to open HID device: VID=0x%04x, PID=0x%04x",
                vendor_id_, product_id_);

    // Open HID device
    hid_device_ = hid_open(vendor_id_, product_id_, nullptr);
    if (!hid_device_) {
      RCLCPP_ERROR(rclcpp::get_logger("HidHardware"),
                   "Failed to open HID device VID=0x%04x, PID=0x%04x. "
                   "Check device connection and permissions (udev rules)",
                   vendor_id_, product_id_);
      // Continue initialization without crashing - interfaces will be empty
      input_report_size_ = 0;
      state_data.assign(0, 0.0);
      command_data.assign(0, 0.0);
      RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                  "Hardware interface will run in disconnected mode");
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Get device info and detect report size
    wchar_t manufacturer[256], product[256];
    hid_get_manufacturer_string(hid_device_, manufacturer, 256);
    hid_get_product_string(hid_device_, product, 256);

    RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                "Connected to HID device: %ls %ls", manufacturer, product);

    // Try to detect input report size by attempting reads of different sizes
    // This is a heuristic approach since hidapi doesn't provide direct report size query
    detect_report_size();

    // Parse type information from parameters
    parse_type_info();

    // Count state + command interfaces
    size_t n_states = 0, n_commands = 0;
    for (const auto & s : info_.sensors) {
      n_states += s.state_interfaces.size();
    }
    for (const auto & g : info_.gpios) {
      n_states   += g.state_interfaces.size();
      n_commands += g.command_interfaces.size();
    }

    state_data.assign(n_states, 0.0);
    command_data.assign(n_commands, 0.0);

    // Allocate dynamic buffer (large enough for maximum possible report)
    input_buffer_.resize(std::max(size_t(64), input_report_size_));

    RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                "HID device initialized: %zu state interfaces (%s), %zu command interfaces (%s)",
                n_states, state_types_str_.c_str(),
                n_commands, command_types_str_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn HidHardware::on_configure
        (const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  std::vector<hardware_interface::StateInterface> HidHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Only export interfaces if HID device was successfully opened
    if (!hid_device_) {
      RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                  "HID device not connected, not exporting any state interfaces");
      return state_interfaces;  // Return empty vector
    }

    size_t i = 0;

    for (const auto & s : info_.sensors) {
      for (const auto & iface : s.state_interfaces) {
        state_interfaces.emplace_back(s.name, iface.name, &state_data[i++]);
      }
    }

    for (const auto & g : info_.gpios) {
      for (const auto & iface : g.state_interfaces) {
        state_interfaces.emplace_back(g.name, iface.name, &state_data[i++]);
      }
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> HidHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Only export interfaces if HID device was successfully opened
    if (!hid_device_) {
      RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                  "HID device not connected, not exporting any command interfaces");
      return command_interfaces;  // Return empty vector
    }

    size_t i = 0;

    for (const auto & g : info_.gpios) {
      for (const auto & iface : g.command_interfaces) {
        command_interfaces.emplace_back(g.name, iface.name, &command_data[i++]);
      }
    }

    return command_interfaces;
  }


  hardware_interface::CallbackReturn HidHardware::on_activate
        (const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn HidHardware::on_deactivate
        (const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type HidHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (!hid_device_) {
      // Device not connected, just return OK without reading
      return hardware_interface::return_type::OK;
    }

    // Use blocking read (timeout = -1 means wait forever, but read() is called at update_rate)
    int res = hid_read(hid_device_, input_buffer_.data(), input_buffer_.size());

    if (res > 0) {
      // HID reports typically start with Report ID as the first byte
      size_t byte_offset = 0;
      uint8_t report_id = 0;

      if (res >= 1) {
        report_id = input_buffer_[0];
        if (report_id != 0) {
          byte_offset = 1;  // Skip report ID byte
        }

        // Log first few reads for debugging
        static int read_count = 0;
        if (read_count < 5) {
          RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                      "Read %d: %d bytes, Report ID: 0x%02x",
                      read_count, res, report_id);
          read_count++;
        }
      }

      // Parse data using type information
      if (!state_data.empty()) {
        state_data[0] = static_cast<double>(report_id);  // First interface is always report_id

        // Parse remaining interfaces based on their types
        size_t interface_idx = 1;  // Start after report_id
        while (interface_idx < state_data.size() && byte_offset < static_cast<size_t>(res)) {
          // Get type for this interface (with bounds checking)
          std::string type = "uint8";  // default
          if (interface_idx < state_types_.size()) {
            type = state_types_[interface_idx];
          }

          size_t type_size = get_type_size(type);

          // Ensure we have enough bytes remaining
          if (byte_offset + type_size <= static_cast<size_t>(res)) {
            state_data[interface_idx] = bytes_to_value(&input_buffer_[byte_offset], type);
            byte_offset += type_size;
          } else {
            // Not enough data remaining
            state_data[interface_idx] = 0.0;
          }

          interface_idx++;
        }

        // Zero out any remaining interfaces if we ran out of data
        for (; interface_idx < state_data.size(); ++interface_idx) {
          state_data[interface_idx] = 0.0;
        }
      }
    } else if (res == 0) {
      // No data available (should not happen in blocking mode)
      static int no_data_count = 0;
      if (no_data_count < 5) {
        RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                    "hid_read returned 0 (no data) - device may not be sending data");
        no_data_count++;
      }
    } else {
      // Error
      static int error_count = 0;
      if (error_count < 5) {
        RCLCPP_ERROR(rclcpp::get_logger("HidHardware"),
                     "hid_read error: %ls", hid_error(hid_device_));
        error_count++;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type HidHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (!hid_device_) {
      // Device not connected, just return OK without writing
      return hardware_interface::return_type::OK;
    }

    if (command_data.empty()) {
      // No commands to write
      return hardware_interface::return_type::OK;
    }

    // Build HID output report
    // Format: [Report ID] [command_data as bytes]
    std::vector<unsigned char> output_buffer;

    // Add report ID (configurable via URDF param)
    uint8_t output_report_id = 1;  // Default output report ID
    if (info_.hardware_parameters.count("output_report_id")) {
      output_report_id = static_cast<uint8_t>(
        std::stoul(info_.hardware_parameters.at("output_report_id"), nullptr, 0));
    }
    output_buffer.push_back(output_report_id);

    // Pack command data as bytes using type information
    for (size_t i = 0; i < command_data.size(); ++i) {
      // Get type for this command interface (with bounds checking)
      std::string type = "float32";  // default
      if (i < command_types_.size()) {
        type = command_types_[i];
      }

      size_t type_size = get_type_size(type);

      // Allocate bytes for this value
      std::vector<unsigned char> value_bytes(type_size);
      value_to_bytes(command_data[i], type, value_bytes.data());

      // Append to output buffer
      output_buffer.insert(output_buffer.end(), value_bytes.begin(), value_bytes.end());
    }

    // Send HID output report
    int res = hid_write(hid_device_, output_buffer.data(), output_buffer.size());

    if (res < 0) {
      static int error_count = 0;
      if (error_count < 5) {
        RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                    "Failed to write HID output report: %ls",
                    hid_error(hid_device_));
        error_count++;
      }
      return hardware_interface::return_type::ERROR;
    }

    // Log first few writes for debugging
    static int write_count = 0;
    if (write_count < 10) {
      RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                  "Write %d: sent %d bytes (Report ID: 0x%02x + %zu float32 values)",
                  write_count, res, output_report_id, command_data.size());
      write_count++;
    }

    return hardware_interface::return_type::OK;
  }

  void HidHardware::parse_type_info() {
    // Parse state types
    if (info_.hardware_parameters.count("state_types")) {
      state_types_str_ = info_.hardware_parameters.at("state_types");
      state_types_ = split_string(state_types_str_, ',');
      RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                  "State types: %s", state_types_str_.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                  "No state_types parameter found, assuming all uint8");
    }

    // Parse command types
    if (info_.hardware_parameters.count("command_types")) {
      command_types_str_ = info_.hardware_parameters.at("command_types");
      command_types_ = split_string(command_types_str_, ',');
      RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                  "Command types: %s", command_types_str_.c_str());
    }
  }

  std::vector<std::string> HidHardware::split_string(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
      tokens.push_back(token);
    }
    return tokens;
  }

  size_t HidHardware::get_type_size(const std::string& type) {
    if (type == "uint8" || type == "int8") return 1;
    if (type == "uint16" || type == "int16") return 2;
    if (type == "uint32" || type == "int32" || type == "float32") return 4;
    if (type == "uint64" || type == "int64" || type == "float64") return 8;
    RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                "Unknown type '%s', assuming 1 byte", type.c_str());
    return 1;
  }

  double HidHardware::bytes_to_value(const unsigned char* bytes, const std::string& type) {
    if (type == "uint8") {
      return static_cast<double>(bytes[0]);
    } else if (type == "int8") {
      return static_cast<double>(static_cast<int8_t>(bytes[0]));
    } else if (type == "uint16") {
      uint16_t val;
      std::memcpy(&val, bytes, 2);
      return static_cast<double>(val);
    } else if (type == "int16") {
      int16_t val;
      std::memcpy(&val, bytes, 2);
      return static_cast<double>(val);
    } else if (type == "uint32") {
      uint32_t val;
      std::memcpy(&val, bytes, 4);
      return static_cast<double>(val);
    } else if (type == "int32") {
      int32_t val;
      std::memcpy(&val, bytes, 4);
      return static_cast<double>(val);
    } else if (type == "float32") {
      float val;
      std::memcpy(&val, bytes, 4);
      return static_cast<double>(val);
    } else if (type == "float64") {
      double val;
      std::memcpy(&val, bytes, 8);
      return val;
    }
    return static_cast<double>(bytes[0]);  // Default: uint8
  }

  void HidHardware::value_to_bytes(double value, const std::string& type, unsigned char* bytes) {
    if (type == "uint8") {
      bytes[0] = static_cast<uint8_t>(value);
    } else if (type == "int8") {
      bytes[0] = static_cast<uint8_t>(static_cast<int8_t>(value));
    } else if (type == "uint16") {
      uint16_t val = static_cast<uint16_t>(value);
      std::memcpy(bytes, &val, 2);
    } else if (type == "int16") {
      int16_t val = static_cast<int16_t>(value);
      std::memcpy(bytes, &val, 2);
    } else if (type == "uint32") {
      uint32_t val = static_cast<uint32_t>(value);
      std::memcpy(bytes, &val, 4);
    } else if (type == "int32") {
      int32_t val = static_cast<int32_t>(value);
      std::memcpy(bytes, &val, 4);
    } else if (type == "float32") {
      float val = static_cast<float>(value);
      std::memcpy(bytes, &val, 4);
    } else if (type == "float64") {
      std::memcpy(bytes, &value, 8);
    } else {
      // Default: uint8
      bytes[0] = static_cast<uint8_t>(value);
    }
  }

  void HidHardware::detect_report_size() {
    // Try different buffer sizes to detect the actual report size
    // This is a heuristic approach since hidapi doesn't provide direct API
    std::vector<size_t> common_sizes = {1, 2, 4, 5, 8, 16, 32, 64};

    // Set non-blocking mode
    hid_set_nonblocking(hid_device_, 1);

    for (size_t size : common_sizes) {
      std::vector<unsigned char> test_buffer(size);
      int res = hid_read(hid_device_, test_buffer.data(), size);
      if (res > 0) {
        input_report_size_ = static_cast<size_t>(res);
        RCLCPP_INFO(rclcpp::get_logger("HidHardware"),
                    "Detected input report size: %zu bytes", input_report_size_);
        break;
      }
    }

    // Fallback to a reasonable default if detection fails
    if (input_report_size_ == 0) {
      input_report_size_ = 8;  // Common size for mice and similar devices
      RCLCPP_WARN(rclcpp::get_logger("HidHardware"),
                  "Could not detect report size, using default: %zu bytes", input_report_size_);
    }

    // Set back to blocking mode for normal operation
    hid_set_nonblocking(hid_device_, 0);
  }


}  // namespace hid_hardware

PLUGINLIB_EXPORT_CLASS(hid_hardware::HidHardware, hardware_interface::SystemInterface)
