// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>

#include <luxi/storage.hpp>
TEST(Storage, HashAndDurableManifest) {
  auto root = luxi::fs::temp_directory_path() / ("luxi-storage-" + std::to_string(luxi::mono()));
  luxi::fs::create_directory(root);
  {
    std::ofstream f(root / "a");
    f << "abc";
  }
  EXPECT_EQ(luxi::hash_file(root / "a"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  auto files = luxi::manifest_files(root);
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files[0]["bytes"], 3);
  luxi::atomic_json(root / "manifest.json", {{"files", files}});
  EXPECT_EQ(luxi::config(root / "manifest.json")["files"], files);
  luxi::fs::remove_all(root);
}
TEST(Storage, IdValidation) {
  EXPECT_FALSE(luxi::simple_id("../foo"));
  EXPECT_FALSE(luxi::simple_id(".partial-x"));
  EXPECT_TRUE(luxi::simple_id("episode-123"));
}
