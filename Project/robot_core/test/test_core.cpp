// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>

#include <luxi/core.hpp>
TEST(Time, ExpiryAndFuture) {
  EXPECT_TRUE(luxi::fresh(100, 200, 100));
  EXPECT_FALSE(luxi::fresh(0, 20));
  EXPECT_FALSE(luxi::fresh(200, 100));
  EXPECT_FALSE(luxi::fresh(100, 201, 100));
}
TEST(Quest, InvalidPacket) {
  EXPECT_THROW(luxi::parse_quest("[]"), std::exception);
  EXPECT_THROW(luxi::parse_quest("{bad"), std::exception);
  EXPECT_THROW(luxi::parse_pose(luxi::Json{
                   {"x", 0}, {"y", 0}, {"z", 0}, {"qw", 0}, {"qx", 0}, {"qy", 0}, {"qz", 0}}),
               std::exception);
}
TEST(Mapping, ReferenceSubtraction) {
  geometry_msgs::msg::Pose p;
  p.orientation.w = 1;
  p.position.x = 2;
  p.position.y = 3;
  p.position.z = 4;
  auto out = luxi::map_pose(p, p);
  EXPECT_NEAR(out.position.x, -.085, 1e-9);
  EXPECT_NEAR(out.position.y, 0, 1e-9);
  EXPECT_NEAR(out.position.z, -.14, 1e-9);
  EXPECT_NEAR(luxi::rot(out).norm(), 1, 1e-9);
}
TEST(Filter, ResetDoesNotKeepOldPose) {
  luxi::PoseFilter f;
  auto p = luxi::pose({0, 0, 0}, Eigen::Quaterniond::Identity());
  f.update(p, 100);
  p.position.x = 1;
  EXPECT_LT(f.update(p, 10000000).position.x, 1);
  f.reset();
  EXPECT_DOUBLE_EQ(f.update(p, 20000000).position.x, 1);
}
