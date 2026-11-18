// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <linux/can.h>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <stdexcept>
#include <algorithm>

namespace luxi::lift {
constexpr uint8_t Hello = 1, Keepalive = 2, Stop = 3, Info = 4, Close = 5, Move = 0x10, Home = 0x11;
inline uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
inline uint32_t u32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline int32_t i32(const uint8_t* p) { auto u = u32(p); int32_t s; std::memcpy(&s, &u, 4); return s; }
inline int16_t i16(const uint8_t* p) { auto u = u16(p); int16_t s; std::memcpy(&s, &u, 2); return s; }
inline can_frame request(int node, uint8_t op, uint8_t sequence, int32_t value = 0, bool uart = false) {
  if (node < 1 || node > 63) throw std::invalid_argument("lift node");
  // HOME uses the persisted reference. No calibration or persistent writes.
  if (!((op == Info && value >= 0 && value <= (uart ? 30 : 24)) ||
        ((op == Hello || op == Keepalive || op == Close) && value != 0) ||
        ((op == Stop || op == Home) && value == 0) ||
        (op == Move && value >= (uart ? -180000 : -100000) && value <= (uart ? 180000 : 200000))))
    throw std::invalid_argument("lift request");
  can_frame f{}; f.can_id = 0x600 + node; f.can_dlc = 8;
  f.data[0] = op; f.data[1] = sequence;
  const auto bits = static_cast<uint32_t>(value);
  for (int i = 0; i < 4; ++i) f.data[2 + i] = (bits >> (8 * i)) & 255;
  f.data[7] = 1;
  return f;
}
struct Sample {
  bool uart = false;
  uint16_t sequence = 0, fault = 0, feedback_age_ms = 65535;
  int32_t current_um = 0, target_um = 0;
  int16_t speed_rpm = 0;
  uint8_t target_status = 0, flags = 0, raw_mask = 0, stable_mask = 0, result = 0, guard = 0;
  uint32_t uptime_ms = 0;
  int64_t first_ns = 0, received_ns = 0;
  std::array<int64_t, 4> page_ns{}, kernel_wall_ns{};
  double age_ms(int64_t now) const { return feedback_age_ms + std::max<int64_t>(0, now - first_ns) / 1e6; }
  bool fresh(int64_t now) const {
    return first_ns > 0 && now >= received_ns && now - first_ns <= 300000000 && age_ms(now) <= 250;
  }
  bool valid(int64_t now) const { return fresh(now) && (flags & 1) && !(flags & 128) && !fault && (uart || !(guard & 4)); }
  // The motor scan age and CAN publication period are separate. Keep the
  // strict 250 ms bound for measurements/planning/arrival, but do not cancel
  // an accepted finite move between scans. Vendor client's watchdog is 500 ms.
  bool healthy_without_position(int64_t now) const {
    return first_ns > 0 && now >= received_ns && now - first_ns <= 300000000 &&
           age_ms(now) <= 500 && !(flags & 128) && !fault && (uart || !(guard & 4));
  }
  bool healthy(int64_t now) const { return healthy_without_position(now) && (flags & 1); }
  bool startup_healthy(int64_t now, int64_t command_started, int32_t origin) const {
    // v29+ permits only this bounded, brake-locked preparation exception.
    return uart && command_started > 0 && now >= command_started && now - command_started < 2000000000LL &&
           first_ns > 0 && now >= received_ns && now - first_ns <= 300000000 && age_ms(now) <= 1500 &&
           (flags & 1) && !(flags & (4 | 128)) && !fault && std::abs(int(speed_rpm)) <= 2 &&
           std::abs(int64_t(current_um) - origin) <= 10;
  }
  bool stationary() const { return !(flags & (4 | 8)) && std::abs(int(speed_rpm)) <= 2; }
};
// Observe assembled, ordered telemetry snapshots only, never timer/status ticks.
class PositionValidity {
  int invalid_ = 0;
  bool seen_valid_ = false;
 public:
  static constexpr int confirmations = 5;
  void observe(const Sample& s) {
    if (s.flags & 1) { invalid_ = 0; seen_valid_ = true; }
    else invalid_ = std::min(invalid_ + 1, confirmations);
  }
  int invalid_count() const { return invalid_; }
  bool pending() const { return seen_valid_ && invalid_ > 0 && invalid_ < confirmations; }
  bool confirmed() const { return invalid_ >= confirmations; }
};
class Assembler {
  struct Group {
    int64_t first = 0;
    uint8_t mask = 0;
    std::array<std::array<uint8_t, 8>, 4> data{};
    std::array<int64_t, 4> times{}, walls{};
  };
  int node_;
  bool uart_;
  std::map<uint16_t, Group> pending_;
  std::optional<std::pair<uint16_t, uint32_t>> last_;
 public:
  uint64_t missing = 0, duplicates = 0, conflicts = 0;
  explicit Assembler(int node, bool uart = false) : node_(node), uart_(uart) {
    if (node < 1 || node > 63) throw std::invalid_argument("lift node");
  }
  void clear_pending() { pending_.clear(); }
  std::optional<Sample> consume(const can_frame& f, int64_t time, int64_t wall) {
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (time - it->second.first > 300000000) { ++missing; it = pending_.erase(it); } else ++it;
    }
    if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) || f.can_dlc != 8) return {};
    int page = -1;
    for (int i = 0; i < 4; ++i) if (f.can_id == canid_t(0x180 + node_ + 0x100 * i)) page = i;
    if (page < 0) return {};
    const auto seq = u16(f.data);
    if (!pending_.count(seq)) {
      if (pending_.size() >= 4) {
        auto oldest = std::min_element(pending_.begin(), pending_.end(),
            [](const auto& a, const auto& b) { return a.second.first < b.second.first; });
        pending_.erase(oldest); ++missing;
      }
      pending_[seq].first = time;
    }
    auto& g = pending_.at(seq);
    if (g.mask & (1 << page)) {
      ++duplicates;
      if (std::memcmp(g.data[page].data(), f.data, 8)) {
        pending_.erase(seq); ++conflicts;
        throw std::runtime_error("conflicting lift telemetry page");
      }
      return {};
    }
    std::copy_n(f.data, 8, g.data[page].begin());
    g.times[page] = time; g.walls[page] = wall; g.mask |= 1 << page;
    if (g.mask != 15) return {};
    Sample s;
    s.uart = uart_;
    s.sequence = seq; s.first_ns = g.first; s.received_ns = time;
    s.page_ns = g.times; s.kernel_wall_ns = g.walls;
    s.current_um = i32(g.data[0].data() + 2); s.speed_rpm = i16(g.data[0].data() + 6);
    s.target_um = i32(g.data[1].data() + 2); s.target_status = g.data[1][6]; s.flags = g.data[1][7];
    s.fault = u16(g.data[2].data() + 2); s.raw_mask = g.data[2][4]; s.stable_mask = g.data[2][5];
    s.result = g.data[2][6]; s.guard = g.data[2][7];
    s.uptime_ms = u32(g.data[3].data() + 2); s.feedback_age_ms = u16(g.data[3].data() + 6);
    pending_.erase(seq);
    if (last_) {
      const uint32_t dt = s.uptime_ms - last_->second;
      const uint16_t ds = s.sequence - last_->first;
      if (ds == 0 && dt == 0) { ++duplicates; return {}; }
      if (dt == 0 || dt >= 0x80000000u || ds == 0 || ds >= 0x8000u) {
        last_.reset(); pending_.clear();
        throw std::runtime_error("lift restarted or telemetry reordered; new operator enable required");
      }
    }
    last_ = std::make_pair(seq, s.uptime_ms);
    return s;
  }
};
}  // namespace luxi::lift
