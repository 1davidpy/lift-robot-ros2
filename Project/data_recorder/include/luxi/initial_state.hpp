// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <luxi/core.hpp>
namespace luxi {
inline int64_t snapshot_ns(const Json& value) {
  if (value.is_string()) {
    const auto text = value.get<std::string>();
    size_t end = 0;
    const auto ns = std::stoll(text, &end);
    if (end != text.size()) throw std::invalid_argument("invalid timestamp");
    return ns;
  }
  if (!value.is_number_integer()) throw std::invalid_argument("invalid timestamp");
  return value.get<int64_t>();
}
inline Json initial_state(const State& arms, const Json& lift, int64_t mono_ns,
                          int64_t ros_ns, const std::string& trigger, int64_t request_ns) {
  const std::vector<double> right(arms.position.begin(), arms.position.begin() + 8);
  const std::vector<double> left(arms.position.begin() + 8, arms.position.end());
  Json platform{{"topic", "/luxi/lift/state"}, {"present", lift.is_object() && !lift.empty()},
                {"valid", false}, {"fresh", false}, {"height_m", nullptr},
                {"height_mm", nullptr}, {"reason", "missing_feedback"}, {"feedback", lift}};
  if (platform["present"].get<bool>()) {
    try {
      const auto received = snapshot_ns(lift.at("receive_mono_ns"));
      const auto first = snapshot_ns(lift.at("receive_first_mono_ns"));
      const auto height_um = lift.at("current_um").get<int32_t>();
      const auto feedback_age = lift.at("feedback_age_ms").get<double>();
      const auto flags = lift.at("flags").get<int>();
      const auto fault = lift.at("fault").get<int>();
      const bool ordered = first > 0 && received >= first && mono_ns >= received;
      const double age_ms = feedback_age + (mono_ns - first) / 1e6;
      const bool recent = ordered && std::isfinite(feedback_age) && feedback_age >= 0 &&
                          mono_ns - first <= 300000000 && age_ms <= 250;
      const bool valid = recent && lift.at("valid").get<bool>() && (flags & 1) &&
                         !(flags & 128) && fault == 0;
      platform.update({{"height_m", height_um * 1e-6}, {"height_mm", height_um * 1e-3},
                       {"current_um", height_um}, {"receive_mono_ns", std::to_string(received)},
                       {"receive_first_mono_ns", std::to_string(first)},
                       {"effective_age_ms_at_capture", age_ms}, {"fresh", recent}, {"valid", valid},
                       {"reason", !recent ? "stale_or_invalid_timestamp" :
                                  !valid ? "invalid_position_or_fault" : "ok"}});
    } catch (const std::exception&) { platform["reason"] = "malformed_feedback"; }
  }
  const bool arms_valid = valid_state(arms) && arms.enabled && fresh(arms.receive_mono_ns, mono_ns);
  return {{"schema", "luxi_initial_state_v1"}, {"trigger", trigger},
          {"captured_mono_ns", std::to_string(mono_ns)}, {"captured_ros_ns", std::to_string(ros_ns)},
          {"request_mono_ns", std::to_string(request_ns)},
          {"time_domain", "board_monotonic_and_ros_wall"},
          {"valid", arms_valid && platform["valid"].get<bool>()},
          {"arms", {{"topic", "/luxi/arms/state"}, {"source", "measured_feedback"},
                    {"order", "right[8],left[8]"}, {"joint_units", "rad"},
                    {"joint_names_per_arm", {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8_gripper"}},
                    {"position", arms.position}, {"right", right}, {"left", left},
                    {"velocity", arms.velocity}, {"valid", arms_valid},
                    {"joint_valid", arms.valid}, {"joint_age_ns", arms.age_ns},
                    {"enabled", arms.enabled}, {"phase", arms.phase},
                    {"receive_mono_ns", std::to_string(arms.receive_mono_ns)},
                    {"age_ms_at_capture", (mono_ns - arms.receive_mono_ns) / 1e6}}},
          {"lift", platform}};
}
} // namespace luxi
