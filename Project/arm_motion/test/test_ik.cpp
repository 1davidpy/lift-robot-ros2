// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>
#include <memory>

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

TEST(Ik, KinematicsMatchesFullForward) {
  const auto path = root + "/device/openarm/openarm_mujoco/v2/cell/cell.xml";
  luxi::ArmConfig config(luxi::config(root + "/Project/robot_bringup/config/arms.json"));
  luxi::Ik ik(path, config);
  char error[1024];
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  const int origin = mj_name2id(model.get(), mjOBJ_SITE, "arm_origin");
  ASSERT_GE(origin, 0);
  for (double offset : {0.0, .05, -.05}) {
    auto joints = config.home;
    for (int side = 0; side < 2; ++side) {
      for (int i = 0; i < 7; ++i) {
        const int k = side * 8 + i;
        joints[k] = std::clamp(joints[k] + offset, config.lo[k], config.hi[k]);
        const auto name = std::string(side ? "openarm_left_" : "openarm_right_") +
                          "joint" + std::to_string(i + 1);
        const int joint = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
        ASSERT_GE(joint, 0);
        data->qpos[model->jnt_qposadr[joint]] = joints[k];
      }
    }
    mj_forward(model.get(), data.get());
    ik.sync(joints);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(
        data->site_xmat + 9 * origin);
    Eigen::Map<const Eigen::Vector3d> position(data->site_xpos + 3 * origin);
    for (int side = 0; side < 2; ++side) {
      const int site = mj_name2id(model.get(), mjOBJ_SITE,
          side ? "left_ee_control_point" : "right_ee_control_point");
      ASSERT_GE(site, 0);
      Eigen::Map<const Eigen::Vector3d> end_position(data->site_xpos + 3 * site);
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> end_rotation(
          data->site_xmat + 9 * site);
      const auto actual = ik.fk(side);
      EXPECT_LT((luxi::pos(actual) - rotation.transpose() * (end_position - position)).norm(), 1e-10);
      EXPECT_LT(luxi::rot(actual).angularDistance(
          Eigen::Quaterniond(rotation.transpose() * end_rotation)), 1e-10);
    }
  }
}
