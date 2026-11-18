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
TEST(Filter, ResetMatchesFreshFilter) {
  luxi::PoseFilter used, fresh;
  auto p = luxi::pose({0, 0, 0}, Eigen::Quaterniond::Identity());
  used.update(p, 1000000000);
  p.position.x = 1;
  used.update(p, 1010000000);
  used.reset();
  p.position.x = 0;
  used.update(p, 1020000000);
  fresh.update(p, 1020000000);
  p.position.x = .1;
  EXPECT_NEAR(used.update(p, 1030000000).position.x,
              fresh.update(p, 1030000000).position.x, 1e-12);
}
TEST(Filter, ResponsiveSettingSettlesPositionAndRotationWithin140ms) {
  luxi::PoseFilter f(4.0);
  auto p = luxi::pose({0, 0, 0}, Eigen::Quaterniond::Identity());
  constexpr int64_t start = 1000000000, dt = 13888889;
  f.update(p, start);
  p.position.x = .1;
  p.orientation = luxi::pose({0, 0, 0},
      Eigen::Quaterniond(Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitZ()))).orientation;
  geometry_msgs::msg::Pose out;
  for (int i = 1; i <= 10; ++i) out = f.update(p, start + i * dt);
  EXPECT_GT(out.position.x, .09);
  EXPECT_LT(luxi::rot(out).angularDistance(luxi::rot(p)), .01);
}

TEST(Mapping, ReferencePitchAndRollDoNotTiltWrists) {
  const auto controller = luxi::pose({.2, .3, .4}, Eigen::Quaterniond::Identity());
  const auto level = luxi::pose({0, 0, 0}, Eigen::Quaterniond::Identity());
  const auto tilted = luxi::pose({0, 0, 0}, Eigen::Quaterniond(
      Eigen::AngleAxisd(.52, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ())));
  const auto expected = luxi::map_pose(controller, level, true);
  const auto corrected = luxi::map_pose(controller, tilted, true);
  EXPECT_NEAR(luxi::rot(expected).angularDistance(luxi::rot(corrected)), 0, 1e-9);
  EXPECT_GT(luxi::rot(expected).angularDistance(
      luxi::rot(luxi::map_pose(controller, tilted, false))), .5);
  EXPECT_NEAR((luxi::pos(corrected) - luxi::pos(luxi::map_pose(controller, tilted, false))).norm(), 0, 1e-12);
}
TEST(Mapping, HeadingAndControllerPitchArePreserved) {
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond pitch(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX()));
  const auto reference = luxi::pose({0, 0, 0}, yaw);
  const auto neutral = luxi::pose({0, 0, 0}, yaw);
  const auto hand = luxi::pose({0, 0, 0}, yaw * pitch);
  EXPECT_NEAR(luxi::rot(luxi::map_pose(neutral, reference, true)).angularDistance(
      luxi::rot(luxi::map_pose(neutral, reference, false))), 0, 1e-9);
  EXPECT_NEAR(luxi::rot(luxi::map_pose(neutral, reference, true)).angularDistance(
      luxi::rot(luxi::map_pose(hand, reference, true))), .4, 1e-9);
}
