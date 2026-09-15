// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>

#include <luxi/core.hpp>
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

#include <luxi/receiver_queue.hpp>
TEST(ReceiverQueue, OnlyVerifiedCommittedRamEpisodesAreReleased) {
  auto root = luxi::fs::temp_directory_path() / ("luxi-queue-" + std::to_string(luxi::mono()));
  luxi::fs::create_directories(root / "episode-test");
  luxi::atomic_json(root / "episode-test/manifest.json", {{"episode_id", "episode-test"}, {"files", luxi::Json::array()}});
  luxi::ReceiverQueue queue(root, true);
  const std::string token(64, 'a'), another(64, 'b');
  luxi::Json claim{{"action", "claim"}, {"token", token}, {"name", "test-pc"}, {"directory", "/test"}};
  ASSERT_TRUE(queue.request(claim, "127.0.0.1")["success"]);
  claim["token"] = another;
  EXPECT_THROW(queue.request(claim, "127.0.0.2"), std::runtime_error);
  luxi::Json receipt{{"action", "verified"}, {"token", token}, {"episode_id", "episode-test"}, {"manifest_sha256", another}, {"release_source", true}};
  EXPECT_THROW(queue.request(receipt, "127.0.0.1"), std::runtime_error);
  ASSERT_TRUE(luxi::fs::exists(root / "episode-test/manifest.json"));
  receipt["manifest_sha256"] = luxi::hash_file(root / "episode-test/manifest.json");
  EXPECT_TRUE(queue.request(receipt, "127.0.0.1")["source_released"]);
  EXPECT_FALSE(luxi::fs::exists(root / "episode-test"));
  EXPECT_TRUE(queue.request(receipt, "127.0.0.1")["already_verified"]);
  EXPECT_TRUE(queue.status()["last_verified"]["source_released"]);
  luxi::fs::remove_all(root);
}
TEST(ReceiverQueue, PersistentStorageCannotBeAutomaticallyDeleted) {
  auto root = luxi::fs::temp_directory_path() / ("luxi-queue-" + std::to_string(luxi::mono()));
  luxi::fs::create_directories(root / "episode-test");
  luxi::atomic_json(root / "episode-test/manifest.json", {{"episode_id", "episode-test"}});
  luxi::ReceiverQueue queue(root, false);
  const std::string token(64, 'a');
  queue.request({{"action", "claim"}, {"token", token}, {"name", "test-pc"}, {"directory", "/test"}}, "127.0.0.1");
  luxi::Json receipt{{"action", "verified"}, {"token", token}, {"episode_id", "episode-test"}, {"manifest_sha256", luxi::hash_file(root / "episode-test/manifest.json")}, {"release_source", true}};
  EXPECT_THROW(queue.request(receipt, "127.0.0.1"), std::runtime_error);
  receipt["release_source"] = false;
  EXPECT_TRUE(queue.request(receipt, "127.0.0.1")["success"]);
  EXPECT_TRUE(luxi::fs::exists(root / "episode-test/manifest.json"));
  luxi::fs::remove_all(root);
}
