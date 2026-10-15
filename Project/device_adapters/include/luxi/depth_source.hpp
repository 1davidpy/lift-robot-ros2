// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace luxi {
// No SDK or ROS types cross this boundary. SDK implementations are loaded
// locally after ROS initializes, isolating bundled vendor DDS symbols.
struct DepthSample {
  int width = 0, height = 0;
  std::vector<uint16_t> pixels;
  float units = 0, fx = 0, fy = 0, cx = 0, cy = 0, distortion[5]{};
  bool calibrated = false;
  uint64_t sequence = 0;
  double device_ms = 0;
  std::string clock_domain;
};
class DepthSource {
 public:
  virtual ~DepthSource() = default;
  // 1=frame, 0=timeout, -1=error. No SDK exceptions cross the plugin boundary.
  virtual int poll(DepthSample&, char* error, size_t capacity) noexcept = 0;
};
using CreateDepthSource = DepthSource* (*)(const char*, int, int, int, char*, size_t);
}
