// SPDX-License-Identifier: Apache-2.0
#include <luxi/transfer.hpp>
#include <luxi/episode_counts.hpp>
#include <openssl/rand.h>
#include <sys/file.h>
#include <atomic>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>
namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
luxi::Json post(const std::string& board, const luxi::Json& body) {
  httplib::Client client(board);
  client.set_connection_timeout(3); client.set_read_timeout(10);
  auto response = client.Post("/api/receiver", body.dump(), "application/json");
  if (!response) throw std::runtime_error("receiver connection unavailable");
  auto result = luxi::Json::parse(response->body);
  if (response->status != 200 || !result.value("success", false))
    throw std::runtime_error(result.value("message", "receiver request rejected"));
  return result;
}
}
int main(int argc, char** argv) {
  try {
    if (argc < 3 || argc > 4 || (argc == 4 && std::string(argv[3]) != "--release-source"))
      throw std::invalid_argument("usage: episode_receive http://board:8000 destination [--release-source]");
    const std::string board = argv[1];
    const bool release = argc == 4;
    luxi::fs::create_directories(argv[2]);
    const auto root = luxi::fs::canonical(argv[2]);
    int lock = open((root / ".receiver.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB)) throw std::runtime_error("destination is in use by another receiver");
    unsigned char random[32];
    if (RAND_bytes(random, sizeof(random)) != 1) throw std::runtime_error("cannot create receiver token");
    std::ostringstream token;
    for (auto c : random) token << std::hex << std::setw(2) << std::setfill('0') << int(c);
    char hostname[201]{};
    if (gethostname(hostname, sizeof(hostname)-1)) throw std::runtime_error("hostname unavailable");
    luxi::Json claim{{"action", "claim"}, {"token", token.str()}, {"name", hostname}, {"directory", root.string()}, {"message", "watching"}};
    claim["episode_counts"] = luxi::episode_counts(root);
    post(board, claim);
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    std::atomic<bool> done{false}, connected{true};
    std::mutex status_mutex;
    std::string episode, message = "watching";
    std::thread heartbeat([&] {
      while (!done) {
        try {
          auto body = claim;
          body["episode_counts"] = luxi::episode_counts(root);
          { std::lock_guard<std::mutex> guard(status_mutex); body["episode"] = episode; body["message"] = message; }
          post(board, body); connected = true;
        } catch (const std::exception&) { connected = false; }
        for (int i = 0; i < 40 && !done; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
    std::cout << "Watching " << board << " -> " << root << "\n"
              << (release ? "Verified episodes will release the X5 RAM copy." : "Source episodes will be retained.") << std::endl;
    std::set<std::string> verified;
    while (!stopped) {
      try {
        if (!connected) throw std::runtime_error("receiver lease unavailable; retrying");
        httplib::Client client(board);
        client.set_connection_timeout(3); client.set_read_timeout(10);
        auto response = client.Get("/api/episodes");
        if (!response || response->status != 200) throw std::runtime_error("episode list unavailable");
        auto episodes = luxi::Json::parse(response->body).get<std::vector<std::string>>();
        std::sort(episodes.begin(), episodes.end());
        for (const auto& id : episodes) {
          if (stopped) break;
          if (verified.count(id)) continue;
          { std::lock_guard<std::mutex> guard(status_mutex); episode = id; message = "transferring"; }
          std::cout << "Receiving " << id << std::endl;
          const auto hash = luxi::receive_episode(board, id, root, [&] { return !stopped && connected.load(); });
          // Data, directories and manifest have been fsynced and atomically committed.
          post(board, {{"action", "verified"}, {"token", token.str()}, {"episode_id", id},
                       {"manifest_sha256", hash}, {"release_source", release}});
          verified.insert(id);
          std::cout << "Verified " << root / id << (release ? " (RAM copy released)" : "") << std::endl;
          { std::lock_guard<std::mutex> guard(status_mutex); episode.clear(); message = "watching"; }
        }
      } catch (const std::exception& e) {
        { std::lock_guard<std::mutex> guard(status_mutex); message = e.what(); }
        std::cerr << e.what() << std::endl;
      }
      for (int i = 0; i < 20 && !stopped; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    done = true; heartbeat.join();
    try { post(board, {{"action", "release"}, {"token", token.str()}}); } catch (...) {}
    close(lock);
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}
