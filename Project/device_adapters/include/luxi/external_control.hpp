// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <nlohmann/json.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace luxi {
// A local ROS command lease, independent of the device UART/CAN session.
// The caller must revoke on ANY driver fault. Late packets cannot re-enable it.
struct ExternalControl {
  std::string owner;
  uint64_t sequence = 0;
  int64_t received_ns = 0;
  bool revoked = true;
  bool live(int64_t now) const {
    return !revoked && received_ns > 0 && now >= received_ns && now - received_ns <= 200000000;
  }
  void revoke() { revoked = true; }
  std::string accept(const nlohmann::json& j, int64_t now) {
    if (!j.is_object() || !j.contains("owner") || !j["owner"].is_string() ||
        !j.contains("sequence") || !j["sequence"].is_number_unsigned() ||
        !j.contains("source_mono_ns") || !j["source_mono_ns"].is_string() ||
        !j.contains("op") || !j["op"].is_string())
      throw std::runtime_error("malformed external command");
    auto candidate = j["owner"].get<std::string>();
    auto seq = j["sequence"].get<uint64_t>();
    const auto stamp_text = j["source_mono_ns"].get<std::string>();
    size_t parsed = 0;
    const int64_t stamp = std::stoll(stamp_text, &parsed);
    const auto op = j["op"].get<std::string>();
    if (candidate.empty() || candidate.size() > 64 || !seq || parsed != stamp_text.size() ||
        stamp <= 0 || stamp > now || now - stamp > 150000000)
      throw std::runtime_error("invalid/stale external command");
    if (op == "arm") {
      if (live(now) || candidate == owner)
        throw std::runtime_error("new explicit owner required after stop");
      owner = candidate; sequence = seq; received_ns = stamp; revoked = false;
      return op;
    }
    if (candidate != owner || seq <= sequence || stamp < received_ns)
      throw std::runtime_error("external owner/sequence mismatch");
    if (op != "stop" && !live(now))
      throw std::runtime_error("external lease expired; no replay");
    sequence = seq; received_ns = stamp;
    return op;
  }
};
}  // namespace luxi
