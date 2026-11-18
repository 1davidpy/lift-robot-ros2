// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
namespace luxi {
// Count finalized local manifests, never RAM queue entries or partial downloads.
inline nlohmann::json episode_counts(const std::filesystem::path& root) {
  nlohmann::json out{{"accepted", 0}, {"failed", 0}, {"cancelled", 0},
                     {"unknown", 0}, {"total", 0}, {"valid", true}};
  try {
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
      const auto id = entry.path().filename().string();
      if (id.rfind("episode-", 0) != 0 || id.size() == 8 ||
          id.find_first_not_of("0123456789", 8) != std::string::npos) continue;
      if (entry.is_symlink() || !entry.is_directory()) continue;
      try {
        const auto path = entry.path() / "manifest.json";
        if (std::filesystem::is_symlink(path)) throw std::runtime_error("symlink manifest");
        std::ifstream in(path); nlohmann::json manifest; in >> manifest;
        if (manifest.at("episode_id") != id) throw std::runtime_error("episode mismatch");
        auto label = manifest.at("status").get<std::string>();
        if (label != "accepted" && label != "failed" && label != "cancelled") label = "unknown";
        out[label] = out[label].get<int>() + 1;
      } catch (...) { out["unknown"] = out["unknown"].get<int>() + 1; }
      out["total"] = out["total"].get<int>() + 1;
    }
  } catch (...) { out["valid"] = false; }
  return out;
}
} // namespace luxi
