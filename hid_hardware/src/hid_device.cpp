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

#include "hid_hardware/hid_device.hpp"
#include <iostream>

namespace hid_hardware
{

// --- HidReportParser Implementation ---
bool HidReportParser::parse(const std::vector<uint8_t> & descriptor)
{
  reports_.clear();
  uint8_t current_report_id = 0;
  uint32_t report_size = 0;
  uint32_t report_count = 0;
  size_t i = 0;

  while (i < descriptor.size())
  {
    uint8_t item = descriptor[i++];
    uint8_t tag = (item >> 4) & 0x0F;
    uint8_t type = (item >> 2) & 0x03;
    uint8_t size = item & 0x03;
    if (size == 3) size = 4;

    uint32_t data = 0;
    if (i + size <= descriptor.size())
    {
      for (uint8_t d = 0; d < size; ++d) data |= (descriptor[i + d] << (d * 8));
    }
    i += size;

    switch (type)
    {
      case 1:  // Global
        if (tag == 8) current_report_id = data;
        else if (tag == 7) report_size = data;
        else if (tag == 9) report_count = data;
        break;
      case 0:  // Main
        if (tag == 8 || tag == 9 || tag == 11)
        {
          HidReportDetails * report = getOrCreateReport(current_report_id);
          if (report->type == ReportType::UNKNOWN)
          {
            if (tag == 8) report->type = ReportType::INPUT;
            else if (tag == 9) report->type = ReportType::OUTPUT;
            else report->type = ReportType::FEATURE;
          }
          report->size_bytes += (report_size * report_count) / 8;
        }
        break;
    }
  }
  return !reports_.empty();
}

const HidReportDetails * HidReportParser::findReport(ReportType type) const
{
  for (const auto & report : reports_)
  {
    if (report.type == type)
    {
      return &report;
    }
  }
  return nullptr;
}

HidReportDetails * HidReportParser::getOrCreateReport(uint8_t id)
{
  for (auto & report : reports_)
  {
    if (report.id == id) return &report;
  }
  reports_.emplace_back();
  reports_.back().id = id;
  return &reports_.back();
}

// --- HidDevice Implementation ---
HidDevice::HidDevice() : handle_(nullptr)
{
  if (hid_init() != 0)
  {
    throw std::runtime_error("Failed to initialize hidapi.");
  }
}

HidDevice::~HidDevice()
{
  disconnect();
  hid_exit();
}

void HidDevice::disconnect()
{
  if (handle_)
  {
    hid_close(handle_);
    handle_ = nullptr;
  }
}

bool HidDevice::connect(int vendor_id, int product_id)
{
  hid_device_info * devs = hid_enumerate(vendor_id, product_id);
  if (!devs)
  {
    return false;
  }
  handle_ = hid_open_path(devs->path);
  hid_free_enumeration(devs);

  return handle_ != nullptr;
}

bool HidDevice::read(std::vector<uint8_t> & data)
{
  if (!handle_) return false;
  data.resize(input_report_info_.size_bytes + 1);
  int res = hid_read_timeout(handle_, data.data(), data.size(), 100);  // 100ms timeout
  return res > 0;
}

bool HidDevice::write(const std::vector<uint8_t> & data)
{
  if (!handle_) return false;
  return hid_write(handle_, data.data(), data.size()) >= 0;
}

}  // namespace hid_hardware
