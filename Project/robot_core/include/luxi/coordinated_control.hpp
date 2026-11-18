// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <luxi/core.hpp>

namespace luxi {
// Host-local coordination lease. This is not a CAN lease or an authentication token.
// Restart/recovery must begin with a new explicit prepare/stop epoch; a repeated
// active/home message cannot restart a failed or interrupted action.
struct CoordinatedControl {
  std::string epoch, phase, control_source = "legacy";
  int64_t sent_ns = 0;
  bool blocked = true;
  bool live(int64_t now) const { return !blocked && fresh(sent_ns, now, 250000000); }
  bool accept(const std::string& text, int64_t now) {
    auto j = Json::parse(text, nullptr, false);
    if (!j.is_object()) return false;
    try {
      auto stamp = std::stoll(j.at("sent_mono_ns").get<std::string>());
      auto next_epoch = j.at("epoch").get<std::string>();
      auto next_phase = j.at("phase").get<std::string>();
      if (!fresh(stamp, now, 250000000) || stamp <= sent_ns || next_epoch.empty()) return false;
      const bool changed = next_epoch != epoch || next_phase != phase;
      if (next_epoch != epoch)
        blocked = next_phase != "prepare" && next_phase != "stop";
      else if (!fresh(sent_ns, now, 250000000)) blocked = true;
      epoch = next_epoch; phase = next_phase; sent_ns = stamp;
      control_source = j.value("control_source", "legacy");
      return changed;
    } catch (const std::exception&) { return false; }
  }
};
inline bool coordinated_status_fresh(const Json& j, int64_t now) {
  try { return j.is_object() && fresh(std::stoll(j.at("receive_mono_ns").get<std::string>()), now, 400000000); }
  catch (const std::exception&) { return false; }
}
// A stopped lift with only its position-valid bit unavailable need not stop
// otherwise healthy arm/base teleoperation. Transport/fault gates stay strict.
inline bool coordinated_lift_position_paused(const Json& j, int64_t now) {
  try {
    if (!coordinated_status_fresh(j, now) ||
        j.value("transport", "") != "uart" ||
        !j.value("position_invalid_isolation", false) ||
        !j.value("position_invalid_paused", false) ||
        j.value("position_invalid_pause_count", 0) != 5 ||
        j.value("position_invalid_limit", 0) != 5 || j.value("shared_failed", true) ||
        j.value("enabled", true)) return false;
    const int flags = j.at("flags").get<int>();
    const int count = j.at("position_invalid_count").get<int>();
    if (j.at("fault").get<int>() || (flags & 128) || count < 0 || count > 5) return false;
    const auto first = std::stoll(j.at("feedback_first_mono_ns").get<std::string>());
    const double age = j.at("feedback_age_ms").get<double>() + (now - first) / 1e6;
    return fresh(first, now, 300000000) && std::isfinite(age) && age >= 0. && age <= 500.;
  } catch (const std::exception&) { return false; }
}
inline bool coordinated_lift_feedback(const Json& j, int64_t now) {
  try {
    if (!j.value("feedback_healthy", false)) return false;
    const auto first = std::stoll(j.at("feedback_first_mono_ns").get<std::string>());
    if (!fresh(first, now, 300000000)) return false;
    const double age = j.at("feedback_age_ms").get<double>() + (now - first) / 1e6;
    if (age <= 500.) return true;
    // The UART motor configuration can briefly suspend its feedback scan.
    // Keep this exception bounded independently of the adapter status bool.
    if (j.value("transport", "can") != "uart" || !j.value("startup_feedback_grace", false)) return false;
    const auto start = std::stoll(j.at("motion_started_mono_ns").get<std::string>());
    const auto phase = j.value("phase", "");
    const int flags = j.at("flags").get<int>();
    return start > 0 && now >= start && now - start < 2000000000LL && age <= 1500. &&
           (phase == "move_ack" || phase == "moving" || phase == "stopping") &&
           (flags & 1) && !(flags & (4 | 128)) && j.at("fault").get<int>() == 0 &&
           std::abs(j.at("speed_rpm").get<int>()) <= 2 &&
           std::abs(j.at("current_um").get<int64_t>() - j.at("startup_origin_um").get<int64_t>()) <= 10;
  } catch (const std::exception&) { return false; }
}
}  // namespace luxi
