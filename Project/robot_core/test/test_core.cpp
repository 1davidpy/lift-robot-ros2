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
TEST(Quest, MissingControllerPoseKeepsButtonsButDisablesTracking) {
  const luxi::Json pose{{"x", 0}, {"y", 0}, {"z", 0},
                        {"qw", 1}, {"qx", 0}, {"qy", 0}, {"qz", 0}};
  // The APK can omit a controller pose while its timestamp and buttons continue.
  luxi::Json packet{{"t", 1.0}, {"rc", pose}, {"lt", 0.0}, {"rt", 0.0},
                    {"v", 0}, {"vl", 0}, {"vr", 0}, {"x", true}};
  const auto q = luxi::parse_quest(packet.dump());
  EXPECT_EQ(q.left_tracking, 2);
  EXPECT_EQ(q.right_tracking, 0);
  EXPECT_TRUE(q.buttons[2]);
  EXPECT_DOUBLE_EQ(q.sender_time, 1.0);
}
TEST(Quest, NullControllerPosesCannotClaimValidTracking) {
  const auto q = luxi::parse_quest(
      R"({"t":1,"lc":null,"rc":null,"lt":0,"rt":0,"v":0,"vl":0,"vr":0})");
  EXPECT_EQ(q.left_tracking, 2);
  EXPECT_EQ(q.right_tracking, 2);
}
TEST(Quest, StaleControllerPosesKeepExistingTrackingSemantics) {
  const luxi::Json pose{{"x", 0.1}, {"y", 0.2}, {"z", 0.3},
                        {"qw", 1}, {"qx", 0}, {"qy", 0}, {"qz", 0}};
  const auto q = luxi::parse_quest(luxi::Json{
      {"t", 1.0}, {"lc", pose}, {"rc", pose}, {"lt", 0.0}, {"rt", 0.0},
      {"v", 1}, {"vl", 1}, {"vr", 1}}.dump());
  EXPECT_EQ(q.left_tracking, 1);
  EXPECT_EQ(q.right_tracking, 1);
  EXPECT_DOUBLE_EQ(q.left.position.x, 0.1);
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
