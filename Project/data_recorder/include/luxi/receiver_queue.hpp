// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <luxi/storage.hpp>
#include <chrono>
#include <mutex>
namespace luxi {
// One receiver lease per queue. Cleanup is allowed only for RAM staging and only
// after the receiver acknowledges the exact committed manifest it verified.
class ReceiverQueue {
  fs::path root_;
  bool ram_;
  std::mutex mutex_;
  std::string token_;
  Json peer_ = Json::object(), last_ = Json::object();
  std::chrono::steady_clock::time_point heartbeat_{};
  bool online() const {
    return !token_.empty() && std::chrono::steady_clock::now() - heartbeat_ < std::chrono::seconds(20);
  }
  static bool digest(const std::string& s) {
    return s.size() == 64 && s.find_first_not_of("0123456789abcdef") == std::string::npos;
  }
 public:
  ReceiverQueue(fs::path root, bool ram) : root_(std::move(root)), ram_(ram) {}
  Json status() {
    std::lock_guard<std::mutex> lock(mutex_);
    return {{"online", online()}, {"peer", peer_}, {"last_verified", last_}, {"release_supported", ram_}};
  }
  Json request(const Json& j, const std::string& address) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto action = j.at("action").get<std::string>();
    const auto token = j.at("token").get<std::string>();
    if (!digest(token)) throw std::runtime_error("invalid receiver token");
    if (action == "claim") {
      if (online() && token_ != token) throw std::runtime_error("another receiver is active; stop it first");
      const auto name = j.at("name").get<std::string>(), directory = j.at("directory").get<std::string>();
      if (name.empty() || name.size() > 200 || directory.empty() || directory.size() > 2000)
        throw std::runtime_error("invalid receiver destination");
      token_ = token;
      heartbeat_ = std::chrono::steady_clock::now();
      peer_ = {{"name", name}, {"address", address}, {"directory", directory},
               {"episode", j.value("episode", "")}, {"message", j.value("message", "watching")}};
      // Optional for older receivers. Counts describe committed files on the PC.
      const auto counts = j.find("episode_counts");
      if (counts != j.end() && counts->is_object()) {
        bool valid = counts->contains("valid") && (*counts)["valid"].is_boolean();
        int64_t sum = 0;
        for (const auto* key : {"accepted", "failed", "cancelled", "unknown", "total"}) {
          const auto item = counts->find(key);
          if (item == counts->end() || !item->is_number_integer() ||
              item->get<int64_t>() < 0 || item->get<int64_t>() > 1000000000) { valid = false; break; }
          if (std::string(key) != "total") sum += item->get<int64_t>();
        }
        if (valid && sum == (*counts)["total"].get<int64_t>()) peer_["episode_counts"] = *counts;
      }
      return {{"success", true}};
    }
    if (!online() || token != token_) throw std::runtime_error("receiver lease expired or owned by another PC");
    if (action == "release") { token_.clear(); return {{"success", true}}; }
    if (action != "verified") throw std::runtime_error("unknown receiver action");
    const auto id = j.at("episode_id").get<std::string>();
    const auto hash = j.at("manifest_sha256").get<std::string>();
    const bool release = j.value("release_source", false);
    if (!simple_id(id) || !digest(hash)) throw std::runtime_error("invalid episode receipt");
    if (release && !ram_) throw std::runtime_error("automatic cleanup is limited to RAM staging");
    const auto receipts = root_ / ".transfer-receipts";
    if (fs::is_symlink(receipts)) throw std::runtime_error("invalid receipt directory");
    fs::create_directories(receipts);
    const auto dir = root_ / id, manifest = dir / "manifest.json", receipt = receipts / (id + ".json");
    if (!fs::exists(dir)) {
      if (fs::is_regular_file(receipt) && !fs::is_symlink(receipt)) {
        std::ifstream in(receipt); Json saved; in >> saved;
        if (saved.value("manifest_sha256", "") == hash) { last_ = saved; return {{"success", true}, {"already_verified", true}}; }
      }
      throw std::runtime_error("episode is unavailable");
    }
    if (fs::is_symlink(dir) || fs::is_symlink(manifest) || !fs::is_regular_file(manifest) || hash_file(manifest) != hash)
      throw std::runtime_error("manifest changed; source retained");
    Json receipt_data{{"episode_id", id}, {"manifest_sha256", hash}, {"receiver", peer_}, {"source_released", release}};
    atomic_json(receipt, receipt_data);
    if (release) {
      // Hide the immutable episode from new downloads before releasing its RAM.
      const auto delivered = root_ / (".delivered-" + id);
      if (fs::exists(delivered)) throw std::runtime_error("cleanup directory already exists");
      fs::rename(dir, delivered);
      fs::remove_all(delivered);
      sync_path(root_);
    }
    last_ = receipt_data;
    return {{"success", true}, {"source_released", release}};
  }
};
}  // namespace luxi
