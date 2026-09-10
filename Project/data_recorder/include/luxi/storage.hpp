// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#pragma once
#include <fcntl.h>
#include <openssl/evp.h>
#include <unistd.h>

#include <filesystem>
#include <iomanip>
#include <luxi/core.hpp>
namespace luxi {
namespace fs = std::filesystem;
inline std::string hash_file(const fs::path& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("hash open failed");
  auto* ctx = EVP_MD_CTX_new();
  if (!ctx) throw std::runtime_error("EVP allocation");
  EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
  char buf[65536];
  while (f) {
    f.read(buf, sizeof(buf));
    EVP_DigestUpdate(ctx, buf, f.gcount());
  }
  if (!f.eof()) {
    EVP_MD_CTX_free(ctx);
    throw std::runtime_error("hash read failed");
  }
  unsigned char digest[32];
  unsigned int size;
  EVP_DigestFinal_ex(ctx, digest, &size);
  EVP_MD_CTX_free(ctx);
  std::ostringstream out;
  for (unsigned int i = 0; i < size; i++)
    out << std::hex << std::setw(2) << std::setfill('0') << int(digest[i]);
  return out.str();
}
inline void sync_path(const fs::path& p) {
  int fd = open(p.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) throw std::runtime_error("fsync open failed");
  int result = fsync(fd);
  close(fd);
  if (result < 0) throw std::runtime_error("fsync failed");
}
inline Json manifest_files(const fs::path& root) {
  Json files = Json::array();
  for (const auto& e : fs::recursive_directory_iterator(root)) {
    if (e.is_symlink()) throw std::runtime_error("symlink in episode");
    if (e.is_regular_file()) {
      sync_path(e.path());
      files.push_back({{"path", fs::relative(e.path(), root).generic_string()},
                       {"bytes", e.file_size()},
                       {"sha256", hash_file(e.path())}});
    }
  }
  return files;
}
inline void atomic_json(const fs::path& path, const Json& j) {
  auto tmp = path;
  tmp += ".tmp";
  {
    std::ofstream f(tmp);
    f << j.dump(2) << '\n';
    f.flush();
    if (!f) throw std::runtime_error("manifest write failed");
  }
  sync_path(tmp);
  fs::rename(tmp, path);
  sync_path(path.parent_path());
}
inline bool simple_id(const std::string& id) {
  return !id.empty() && id.size() < 120 &&
         id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
             std::string::npos;
}
}  // namespace luxi
