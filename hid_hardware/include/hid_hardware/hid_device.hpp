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

#ifndef HID_HARDWARE__HID_DEVICE_HPP_
#define HID_HARDWARE__HID_DEVICE_HPP_

#include <hidapi/hidapi.h>

#include <string>
#include <vector>

namespace hid_hardware
{
// Defines the type of a report (Input, Output, or Feature)
enum class ReportType { INPUT, OUTPUT, FEATURE, UNKNOWN };

// A structure to hold the parsed details for a single HID report
struct HidReportDetails
{
  uint8_t id = 0;
  ReportType type = ReportType::UNKNOWN;
  uint16_t size_bytes = 0;
};

// A helper class to parse the HID report descriptor
class HidReportParser
{
public:
  bool parse(const std::vector<uint8_t> & descriptor);
  const HidReportDetails * findReport(ReportType type) const;

private:
  std::vector<HidReportDetails> reports_;
  HidReportDetails * getOrCreateReport(uint8_t id);
};

// A class to encapsulate the HID device communication
class HidDevice
{
public:
  HidDevice();
  ~HidDevice();

  bool connect(int vendor_id, int product_id);
  void disconnect();

  bool read(std::vector<uint8_t> & data);
  bool write(const std::vector<uint8_t> & data);

  const HidReportDetails & getInputReportDetails() const { return input_report_info_; }
  const HidReportDetails & getOutputReportDetails() const { return output_report_info_; }

private:
  hid_device * handle_;
  HidReportParser parser_;
  HidReportDetails input_report_info_;
  HidReportDetails output_report_info_;
};

}  // namespace hid_hardware

#endif  // HID_HARDWARE__HID_DEVICE_HPP_
