#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace luxi {
// Slow, bounded position-error integration. This is not an encoder zero offset
// or model-based gravity feedforward. Grippers never participate.
class StaticPositionCompensation {
 public:
  using Axes = std::array<double, 16>;
  static constexpr double max_bias = .2, max_rate = .03, gain = .5;
  static constexpr double deadband = .003, settle_seconds = .3;
  Axes bias{}, stationary_seconds{};

  void reset() {
    bias.fill(0); stationary_seconds.fill(0); initialized_ = false;
  }
  void freeze() { stationary_seconds.fill(0); }
  Axes apply(const Axes& nominal, const Axes& position, const Axes& velocity,
             const Axes& lower, const Axes& upper, const Axes& speed,
             double dt, bool allow_learning) {
    dt = std::clamp(dt, 0., .05);
    Axes output = nominal;
    for (int i = 0; i < 16; ++i) {
      if (i % 8 == 7) {
        bias[i] = stationary_seconds[i] = 0;
        previous_[i] = nominal[i]; continue;
      }
      const bool nominal_valid = std::isfinite(nominal[i]) && nominal[i] >= lower[i] && nominal[i] <= upper[i];
      if (!nominal_valid) {
        stationary_seconds[i] = 0; previous_[i] = nominal[i]; continue;
      }
      const double low = std::max(-max_bias, lower[i] - nominal[i]);
      const double high = std::min(max_bias, upper[i] - nominal[i]);
      bias[i] = std::clamp(bias[i], low, high); // Anti-windup at original limits.
      const double error = nominal[i] - position[i];
      if (!initialized_ || !allow_learning || !std::isfinite(position[i]) || !std::isfinite(velocity[i]) || std::abs(nominal[i] - previous_[i]) > .00001 ||
          std::abs(velocity[i]) > .08 || std::abs(error) > .25) {
        stationary_seconds[i] = 0;
      } else {
        stationary_seconds[i] += dt;
        if (stationary_seconds[i] >= settle_seconds && std::abs(error) > deadband) {
          const double step = std::min(max_rate, speed[i]) * dt;
          bias[i] = std::clamp(bias[i] + std::clamp(gain * error * dt, -step, step), low, high);
        }
      }
      previous_[i] = nominal[i];
      output[i] = std::clamp(nominal[i] + bias[i], lower[i], upper[i]);
    }
    initialized_ = true;
    return output;
  }

 private:
  Axes previous_{};
  bool initialized_ = false;
};
}  // namespace luxi
