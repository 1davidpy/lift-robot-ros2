// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>

#include <luxi/ik.hpp>
static std::string root = LUXI_ROOT;
TEST(Ik, NamedModelAndConvergence) {
  luxi::ArmConfig c(luxi::config(root + "/Project/robot_bringup/config/arms.json"));
  luxi::Ik ik(root + "/device/openarm/openarm_mujoco/v2/cell/cell.xml", c);
  luxi::Targets t;
  t.right = ik.fk(0);
  t.left = ik.fk(1);
  t.right.position.z += .03;
  t.left.position.z += .03;
  t.grippers = {-.5, .5};
  auto q = c.home;
  double before = (luxi::pos(ik.fk(0)) - luxi::pos(t.right)).norm();
  for (int n = 0; n < 120; n++) {
    auto next = ik.solve(t, q);
    for (int i = 0; i < 16; i++) {
      EXPECT_TRUE(std::isfinite(next[i]));
      EXPECT_LE(std::abs(next[i] - q[i]), c.speed[i] * .033 + 1e-8);
      if (i % 8 != 7) {
        EXPECT_GE(next[i], c.lo[i] - 1e-8);
        EXPECT_LE(next[i], c.hi[i] + 1e-8);
      }
    }
    q = next;
  }
  ik.sync(q);
  EXPECT_LT((luxi::pos(ik.fk(0)) - luxi::pos(t.right)).norm(), before * .6);
  EXPECT_LT((luxi::pos(ik.fk(1)) - luxi::pos(t.left)).norm(), .02);
}
TEST(Ik, MissingModelRejected) {
  luxi::ArmConfig c(luxi::config(root + "/Project/robot_bringup/config/arms.json"));
  EXPECT_THROW(luxi::Ik("/nonexistent/model.xml", c), std::runtime_error);
}
