// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <luxi/core.hpp>
#include <luxi_interfaces/msg/rgbd_frame.hpp>
#include <algorithm>

namespace luxi {
inline bool valid_rgbd(const luxi_interfaces::msg::RGBDFrame& p, const std::string& id) {
  const auto& c = p.color; const auto& d = p.depth; const auto& i = p.camera_info;
  const auto colon = p.pair_id.find(':');
  return p.sequence > 0 && p.camera_id == id && c.camera_id == id && d.camera_id == id &&
      colon != std::string::npos && colon > 0 && p.pair_id.size() < 64 &&
      p.pair_id.substr(colon + 1) == std::to_string(p.sequence) &&
      p.pair_id.substr(0, colon).find_first_not_of("0123456789") == std::string::npos &&
      p.header.frame_id == id + "_color_optical" &&
      p.header == c.header && p.header == d.header && p.header == i.header &&
      p.sequence == c.sequence && p.sequence == d.sequence &&
      c.receive_mono_ns == d.receive_mono_ns &&
      (c.timestamp_quality == "mock_host") == (d.timestamp_quality == "mock_host") &&
      d.width > 0 && d.width <= 1920 && d.height > 0 && d.height <= 1080 &&
      c.width == d.width && c.height == d.height && i.width == d.width && i.height == d.height &&
      c.encoding == "jpeg" && d.encoding == "32FC1" &&
      d.data.size() == size_t(d.width) * d.height * 4 &&
      c.data.size() >= 4 && c.data.size() <= size_t(c.width) * c.height * 4 + 65536 &&
      c.data[0] == 0xff && c.data[1] == 0xd8 &&
      c.data[c.data.size()-2] == 0xff && c.data.back() == 0xd9;
}
inline Json rgbd_metadata(const luxi_interfaces::msg::RGBDFrame& p) {
  const auto& d = p.depth; const auto& c = p.color; const auto& i = p.camera_info;
  const bool calibrated = !i.distortion_model.empty() && i.k[0] > 0 && i.k[4] > 0 &&
      std::all_of(i.k.begin(), i.k.end(), [](double v) { return std::isfinite(v); }) &&
      std::all_of(i.r.begin(), i.r.end(), [](double v) { return std::isfinite(v); }) &&
      std::all_of(i.p.begin(), i.p.end(), [](double v) { return std::isfinite(v); }) &&
      std::all_of(i.d.begin(), i.d.end(), [](double v) { return std::isfinite(v); });
  Json j = {{"schema", "luxi_rgbd_v1"}, {"camera_id", p.camera_id}, {"pair_id", p.pair_id},
      {"sequence", std::to_string(p.sequence)}, {"aligned_to_color", true},
      {"frame_id", p.header.frame_id}, {"width", d.width}, {"height", d.height},
      {"stamp", {{"sec", p.header.stamp.sec}, {"nanosec", p.header.stamp.nanosec}}},
      {"receive_mono_ns", std::to_string(d.receive_mono_ns)},
      {"age_ms", (mono() - d.receive_mono_ns) / 1000000},
      {"mock", d.timestamp_quality == "mock_host"}, {"hardware_synchronized", false},
      {"pairing", "sdk_frameset"}, {"calibration_available", calibrated},
      {"ros_projection_compatible", i.distortion_model == "plumb_bob"},
      {"color", {{"encoding", "jpeg"}, {"bytes", c.data.size()},
          {"device_sequence", std::to_string(p.color_device_sequence)},
          {"device_timestamp_ns", std::to_string(c.device_timestamp)}, {"timestamp_quality", c.timestamp_quality}}},
      {"depth", {{"encoding", "32FC1"}, {"unit", "metre"}, {"byte_order", "little-endian"},
          {"step", d.width * 4}, {"invalid_value", "NaN"}, {"bytes", d.data.size()},
          {"value_semantics", "source_depth_z_resampled_to_color"},
          {"device_sequence", std::to_string(p.depth_device_sequence)},
          {"device_timestamp_ns", std::to_string(d.device_timestamp)}, {"timestamp_quality", d.timestamp_quality}}}};
  if (calibrated) j["intrinsics"] = {{"distortion_model", i.distortion_model},
      {"K", i.k}, {"D", i.d}, {"R", i.r}, {"P", i.p}};
  return j;
}
}
