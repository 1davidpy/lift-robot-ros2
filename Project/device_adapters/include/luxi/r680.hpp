// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <linux/can.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
namespace luxi::r680 {
using Velocity = std::array<double, 3>;
inline int16_t be16(const uint8_t* p) {
  int value = (int(p[0]) << 8) | p[1];
  return static_cast<int16_t>(value >= 32768 ? value - 65536 : value);
}
inline void put16(uint8_t* p, double value, double scale) {
  if (!std::isfinite(value) || value * scale < -32768 || value * scale > 32767)
    throw std::invalid_argument("R680 int16 range");
  auto raw = static_cast<uint16_t>(static_cast<int16_t>(std::lround(value * scale)));
  p[0] = raw >> 8;
  p[1] = raw & 255;
}
inline can_frame encode(const Velocity& v) {
  can_frame f{};
  f.can_id = 0x181;
  f.can_dlc = 8;
  for (int i = 0; i < 3; ++i) put16(f.data + 2 * i, v[i], 1000);
  return f;
}
struct Feedback {
  Velocity velocity{}, acceleration{}, gyro{};
  double voltage = 0;
  uint8_t stop = 0;
  std::array<int64_t, 3> mono_ns{};
  std::array<uint8_t, 24> bytes{};
};
class Decoder {
  int next_ = 0;
  std::array<uint8_t, 24> packet_{};
  std::array<int64_t, 3> times_{};

 public:
  uint64_t rejected = 0, accepted = 0;
  std::optional<Feedback> consume(const can_frame& f, int64_t t) {
    if (f.can_id < 0x101 || f.can_id > 0x103 || f.can_dlc != 8) {
      next_ = 0;
      ++rejected;
      return {};
    }
    int part = f.can_id - 0x101;
    if (part == 0) {
      if (next_) ++rejected;
      next_ = 0;
    }
    if (part != next_ || (part && (t < times_[part - 1] || t - times_[0] > 20000000))) {
      next_ = 0;
      ++rejected;
      return {};
    }
    std::copy_n(f.data, 8, packet_.begin() + part * 8);
    times_[part] = t;
    if (++next_ != 3) return {};
    next_ = 0;
    uint8_t bcc = 0;
    for (int i = 0; i < 22; ++i) bcc ^= packet_[i];
    if (packet_[0] != 0x7b || packet_[23] != 0x7d || bcc != packet_[22]) {
      ++rejected;
      return {};
    }
    Feedback s;
    s.stop = packet_[1];
    s.bytes = packet_;
    s.mono_ns = times_;
    for (int i = 0; i < 3; ++i) {
      s.velocity[i] = be16(packet_.data() + 2 + 2 * i) / 1000.0;
      s.acceleration[i] = be16(packet_.data() + 8 + 2 * i) / 1671.84;
      s.gyro[i] = be16(packet_.data() + 14 + 2 * i) / 3754.9;
    }
    s.voltage = be16(packet_.data() + 20) / 1000.0;
    ++accepted;
    return s;
  }
};
struct Mapping {
  double deadzone = .16, linear = .2, angular = .3141592653589793;
  bool reverse_steering = true;
  Velocity map(const std::array<double, 4>& sticks) const {
    for (double x : sticks)
      if (!std::isfinite(x) || std::abs(x) > 1) throw std::invalid_argument("Quest stick range");
    auto axis = [this](double x) { return std::abs(x) < deadzone ? 0.0 : x; };
    Velocity v{{axis(sticks[1]) * linear, -axis(sticks[0]) * linear, -axis(sticks[2]) * angular}};
    // STM32 PS2_control() reverses yaw while moving backwards (non-Ackermann).
    if (reverse_steering && v[0] < 0) v[2] = -v[2];
    return v;
  }
};
inline bool fresh(int64_t source, int64_t now, int64_t limit) {
  return source > 0 && now >= source && now - source <= limit;
}
class Gate {
 public:
  Mapping mapping;
  bool enabled = false, tracking = false, menu = false;
  bool require_left_grip = true;
  double grip = 0;
  int64_t quest_time = 0, feedback_time = 0;
  uint8_t stop = 1;
  std::array<double, 4> sticks{};
  std::string reason = "disabled";
  bool input_valid() const {
    return std::isfinite(grip) && grip >= 0 && grip <= 1 &&
           std::all_of(sticks.begin(), sticks.end(),
                       [](double x) { return std::isfinite(x) && std::abs(x) <= 1; });
  }
  bool start(int64_t now) {
    if (!input_valid() || !fresh(quest_time, now, 200000000) ||
        !fresh(feedback_time, now, 100000000) || !tracking || menu || stop ||
        (require_left_grip && grip > .2))
      return false;
    auto v = mapping.map(sticks);
    if (v[0] != 0 || v[1] != 0 || v[2] != 0) return false;
    enabled = true;
    reason = "ready";
    return true;
  }
  void halt(const std::string& why) {
    enabled = false;
    reason = why;
  }
  Velocity command(int64_t now) {
    if (!enabled) return {};
    if (!input_valid())
      halt("invalid_input");
    else if (!fresh(quest_time, now, 200000000))
      halt("quest_timeout");
    else if (!tracking)
      halt("tracking_invalid");
    else if (menu)
      halt("recording_menu");
    else if (!fresh(feedback_time, now, 250000000))
      halt("feedback_timeout");
    else if (stop)
      halt("chassis_disabled");
    if (!enabled) return {};
    if (require_left_grip && grip < .7) {
      reason = "deadman_released";
      return {};
    }
    if (!fresh(feedback_time, now, 100000000)) {
      reason = "feedback_delayed";
      return {};
    }
    const auto wanted = mapping.map(sticks);
    // Centering is an explicit stop: bypass the adapter's acceleration ramp.
    reason = wanted == Velocity{} ? "sticks_centered" : "teleop";
    return wanted;
  }
};
}  // namespace luxi::r680
