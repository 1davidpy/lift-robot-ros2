// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.



// clang-format off
// Eigen must precede the _res resolver macro exposed by httplib on Ubuntu 22.04.
#include <Eigen/Core>
#include <httplib.h>
// clang-format on
#include <luxi/storage.hpp>
int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::invalid_argument("usage: episode_pull http://board:8000 episode-id destination");
    std::string id = argv[2];
    if (!luxi::simple_id(id)) throw std::invalid_argument("episode id");
    httplib::Client client(argv[1]);
    client.set_connection_timeout(5);
    client.set_read_timeout(30);
    auto response = client.Get(("/api/episodes/" + id + "/manifest").c_str());
    if (!response || response->status != 200) throw std::runtime_error("manifest unavailable");
    auto m = luxi::Json::parse(response->body);
    if (m.at("episode_id") != id) throw std::runtime_error("episode mismatch");
    luxi::fs::path root = argv[3];
    luxi::fs::create_directories(root);
    root = luxi::fs::canonical(root);
    auto pending = root / (".partial-" + id), final = root / id;
    if (luxi::fs::is_symlink(pending) || luxi::fs::is_symlink(final))
      throw std::runtime_error("symlink destination");
    bool exists = luxi::fs::exists(final);
    auto dir = exists ? final : pending;
    luxi::fs::create_directories(dir);
    for (const auto& file : m.at("files")) {
      std::string name = file.at("path"), hash = file.at("sha256");
      uint64_t length = file.at("bytes");
      luxi::fs::path relative = name;
      if (relative.is_absolute() || relative.empty() ||
          name.find_first_not_of(
              "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./-") !=
              std::string::npos)
        throw std::runtime_error("unsafe manifest path");
      for (const auto& part : relative)
        if (part == ".." || part == ".") throw std::runtime_error("unsafe manifest path");
      auto path = dir / relative;
      auto walk = dir;
      for (const auto& part : relative) {
        walk /= part;
        if (luxi::fs::is_symlink(walk)) throw std::runtime_error("symlink destination component");
      }
      luxi::fs::create_directories(path.parent_path());
      uint64_t offset = luxi::fs::exists(path) ? luxi::fs::file_size(path) : 0;
      if (offset == length && luxi::hash_file(path) == hash) continue;
      if (exists) throw std::runtime_error("existing committed episode differs");
      if (offset >= length) {
        std::ofstream clear(path, std::ios::binary | std::ios::trunc);
        offset = 0;
      }
      std::ofstream out(path, std::ios::binary | std::ios::app);
      if (!out) throw std::runtime_error("download destination");
      httplib::Headers headers;
      if (offset) headers.emplace("Range", "bytes=" + std::to_string(offset) + "-");
      uint64_t received = offset;
      auto result = client.Get(
          ("/api/files/" + id + "/" + name).c_str(), headers,
          [&](const httplib::Response& r) { return r.status == (offset ? 206 : 200); },
          [&](const char* data, size_t size) {
            if (size > length - received) return false;
            out.write(data, size);
            received += size;
            return bool(out);
          });
      out.flush();
      out.close();
      if (!result || received != length)
        throw std::runtime_error("download interrupted; partial retained: " + name);
      if (luxi::hash_file(path) != hash) {
        luxi::fs::resize_file(path, 0);
        throw std::runtime_error("checksum mismatch: " + name);
      }
      luxi::sync_path(path);
    }
    if (!exists) {
      luxi::atomic_json(dir / "manifest.json", m);
      for (const auto& e : luxi::fs::recursive_directory_iterator(dir))
        if (e.is_directory()) luxi::sync_path(e.path());
      luxi::sync_path(dir);
      luxi::fs::rename(dir, final);
      luxi::sync_path(root);
    }
    std::cout << "Verified episode: " << final << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
