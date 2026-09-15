// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <httplib.h>
#include <luxi/storage.hpp>
#include <functional>
#include <set>
namespace luxi {
inline std::string receive_episode(const std::string& board, const std::string& id,
                                   const fs::path& destination,
                                   const std::function<bool()>& keep_running = [] { return true; }) {
    if (!luxi::simple_id(id)) throw std::invalid_argument("episode id");
    httplib::Client client(board);
    client.set_connection_timeout(5);
    client.set_read_timeout(30);
    auto response = client.Get(("/api/episodes/" + id + "/manifest").c_str());
    if (!response || response->status != 200) throw std::runtime_error("manifest unavailable");
    auto m = luxi::Json::parse(response->body);
    if (m.at("episode_id") != id) throw std::runtime_error("episode mismatch");
    luxi::fs::path root = destination;
    luxi::fs::create_directories(root);
    root = luxi::fs::canonical(root);
    auto pending = root / (".partial-" + id), final = root / id;
    if (luxi::fs::is_symlink(pending) || luxi::fs::is_symlink(final))
      throw std::runtime_error("symlink destination");
    bool exists = luxi::fs::exists(final);
    auto dir = exists ? final : pending;
    luxi::fs::create_directories(dir);
    if (luxi::fs::is_symlink(dir / "manifest.json")) throw std::runtime_error("symlink manifest");
    std::set<std::string> names;
    for (const auto& file : m.at("files")) {
      if (!keep_running()) throw std::runtime_error("receiver stopped; partial retained");
      std::string name = file.at("path"), hash = file.at("sha256");
      if (name == "manifest.json" || hash.size() != 64 || hash.find_first_not_of("0123456789abcdef") != std::string::npos)
        throw std::runtime_error("invalid manifest file");
      uint64_t length = file.at("bytes");
      luxi::fs::path relative = name;
      if (!names.insert(name).second || relative.is_absolute() || relative.empty() ||
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
      if (luxi::fs::exists(path) && offset == length && luxi::hash_file(path) == hash) {
        luxi::sync_path(path);
        continue;
      }
      if (exists) throw std::runtime_error("existing committed episode differs");
      if (offset >= length) {
        std::ofstream clear(path, std::ios::binary | std::ios::trunc);
        offset = 0;
      }
      std::ofstream out(path, std::ios::binary | std::ios::app);
      if (!out) throw std::runtime_error("download destination");
      httplib::Headers headers;
      if (offset) headers.emplace("Range", "bytes=" + std::to_string(offset) + "-");
      if (luxi::fs::space(root).available < length - offset + 268435456ULL)
        throw std::runtime_error("receiver disk full; source retained");
      uint64_t received = offset;
      auto result = client.Get(
          ("/api/files/" + id + "/" + name).c_str(), headers,
          [&](const httplib::Response& r) { return r.status == (offset ? 206 : 200); },
          [&](const char* data, size_t size) {
            if (!keep_running() || size > length - received) return false;
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
      {
        std::ofstream out(dir / "manifest.json", std::ios::binary | std::ios::trunc);
        out << response->body; out.flush();
        if (!out) throw std::runtime_error("manifest write failed");
      }
      luxi::sync_path(dir / "manifest.json");
      for (const auto& e : luxi::fs::recursive_directory_iterator(dir))
        if (e.is_directory()) luxi::sync_path(e.path());
      luxi::sync_path(dir);
      luxi::fs::rename(dir, final);
      luxi::sync_path(root);
    }
    else {
      if (luxi::fs::is_symlink(final / "manifest.json")) throw std::runtime_error("symlink manifest");
      std::ifstream in(final / "manifest.json"); std::ostringstream saved; saved << in.rdbuf();
      if (!in || saved.str() != response->body) throw std::runtime_error("existing episode manifest differs");
    }
    return luxi::hash_file(final / "manifest.json");
}
}  // namespace luxi
