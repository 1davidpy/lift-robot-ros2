#include <luxi/episode_counts.hpp>
#include <luxi/receiver_queue.hpp>
#include <cassert>
#include <unistd.h>
int main() {
  const auto root = std::filesystem::temp_directory_path() / ("luxi-count-test-" + std::to_string(getpid()));
  std::filesystem::create_directory(root);
  auto write = [&](const std::string& id, const std::string& label) {
    std::filesystem::create_directory(root / id);
    std::ofstream(root / id / "manifest.json") << nlohmann::json{{"episode_id",id},{"status",label}};
  };
  write("episode-1", "accepted"); write("episode-2", "failed"); write("episode-3", "cancelled");
  write(".partial-episode-4", "accepted"); write("episode-5", "unexpected");
  std::filesystem::create_directory_symlink(root / "episode-1", root / "episode-6");
  auto counts = luxi::episode_counts(root);
  assert(counts["total"] == 4 && counts["accepted"] == 1 && counts["failed"] == 1);
  assert(counts["cancelled"] == 1 && counts["unknown"] == 1 && counts["valid"] == true);
  luxi::ReceiverQueue queue(root, false);
  luxi::Json claim{{"action","claim"},{"token",std::string(64,'a')},{"name","test"},{"directory",root.string()},{"episode_counts",counts}};
  queue.request(claim,"127.0.0.1"); assert(queue.status()["peer"]["episode_counts"] == counts);
  claim["episode_counts"]["total"] = 100;
  queue.request(claim,"127.0.0.1"); assert(!queue.status()["peer"].contains("episode_counts"));
  claim.erase("episode_counts"); queue.request(claim,"127.0.0.1");
  assert(!queue.status()["peer"].contains("episode_counts"));
  std::filesystem::remove_all(root / "episode-1");
  assert(luxi::episode_counts(root)["accepted"] == 0);
  std::filesystem::remove_all(root);
  assert(luxi::episode_counts(root)["valid"] == false);
}
