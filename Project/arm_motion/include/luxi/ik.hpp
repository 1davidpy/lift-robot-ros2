// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#pragma once
#include <mujoco/mujoco.h>

#include <luxi/core.hpp>
namespace luxi {
class Ik {
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int origin_;
  std::array<int, 2> site_;
  std::array<int, 14> joints_;
  ArmConfig cfg_;

 public:
  Ik(const std::string& path, const ArmConfig& cfg) : cfg_(cfg) {
    char error[1024];
    m_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!m_) throw std::runtime_error(error);
    d_ = mj_makeData(m_);
    try {
      origin_ = id(mjOBJ_SITE, "arm_origin");
      for (int s = 0; s < 2; s++) {
        site_[s] = id(mjOBJ_SITE, s ? "left_ee_control_point" : "right_ee_control_point");
        for (int i = 0; i < 7; i++)
          joints_[7 * s + i] = id(mjOBJ_JOINT, std::string(s ? "openarm_left_" : "openarm_right_") +
                                                   "joint" + std::to_string(i + 1));
      }
      sync(cfg.home);
    } catch (...) {
      mj_deleteData(d_);
      mj_deleteModel(m_);
      throw;
    }
  }
  ~Ik() {
    mj_deleteData(d_);
    mj_deleteModel(m_);
  }
  int id(mjtObj type, const std::string& name) {
    int v = mj_name2id(m_, type, name.c_str());
    if (v < 0) throw std::runtime_error("Model missing " + name);
    return v;
  }
  void sync(const std::array<double, 16>& q) {
    for (int k = 0; k < 14; k++) d_->qpos[m_->jnt_qposadr[joints_[k]]] = q[(k / 7) * 8 + k % 7];
    mj_forward(m_, d_);
  }
  geometry_msgs::msg::Pose fk(int side) {
    mj_forward(m_, d_);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(d_->site_xmat + 9 * origin_);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> e(d_->site_xmat +
                                                                     9 * site_[side]);
    Eigen::Map<const Eigen::Vector3d> o(d_->site_xpos + 3 * origin_),
        p(d_->site_xpos + 3 * site_[side]);
    return pose(r.transpose() * (p - o), Eigen::Quaterniond(r.transpose() * e));
  }
  std::array<double, 16> solve(const Targets& target, const std::array<double, 16>& seed,
                               double dt = .033) {
    sync(seed);
    auto out = seed;
    double h = dt / 2;
    for (int step = 0; step < 2; step++)
      for (int side = 0; side < 2; side++) {
        mj_forward(m_, d_);
        auto desired = side ? target.left : target.right;
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> origin_r(d_->site_xmat +
                                                                                9 * origin_),
            current_r(d_->site_xmat + 9 * site_[side]);
        Eigen::Map<const Eigen::Vector3d> origin_p(d_->site_xpos + 3 * origin_),
            current_p(d_->site_xpos + 3 * site_[side]);
        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = origin_r * pos(desired) + origin_p - current_p;
        Eigen::AngleAxisd aa(origin_r * rot(desired).toRotationMatrix() * current_r.transpose());
        error.tail<3>() = aa.axis() * aa.angle();
        // Limit task velocity, then solve a box-constrained damped least-squares QP.
        error.head<3>() *= std::min(1.0, .5 * dt / std::max(error.head<3>().norm(), 1e-9));
        error.tail<3>() *= std::min(1.0, 2.0 * dt / std::max(error.tail<3>().norm(), 1e-9));
        std::vector<double> jp(3 * m_->nv), jr(3 * m_->nv);
        mj_jacSite(m_, d_, jp.data(), jr.data(), site_[side]);
        Eigen::Matrix<double, 6, 7> jac;
        Eigen::Matrix<double, 7, 1> lo, hi, q;
        for (int i = 0; i < 7; i++) {
          int joint = joints_[side * 7 + i], v = m_->jnt_dofadr[joint], k = side * 8 + i;
          q[i] = d_->qpos[m_->jnt_qposadr[joint]];
          for (int r = 0; r < 3; r++) {
            jac(r, i) = jp[r * m_->nv + v];
            jac(r + 3, i) = jr[r * m_->nv + v];
          }
          lo[i] = std::max(-cfg_.speed[k], (cfg_.lo[k] - q[i]) / h);
          hi[i] = std::min(cfg_.speed[k], (cfg_.hi[k] - q[i]) / h);
          if (lo[i] > hi[i]) throw std::runtime_error("IK seed outside limits");
        }
        Eigen::Matrix<double, 7, 7> H =
            jac.transpose() * jac + .1 * Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 7, 1> b = jac.transpose() * (error / dt),
                                    v = Eigen::Matrix<double, 7, 1>::Zero();
        for (int i = 0; i < 7; i++) b[i] += .01 * (cfg_.home[side * 8 + i] - q[i]);
        // Cyclic coordinate descent solves the strictly convex bounded QP without Python.
        for (int pass = 0; pass < 80; pass++) {
          double change = 0;
          for (int i = 0; i < 7; i++) {
            double n = std::clamp(v[i] + (b[i] - H.row(i).dot(v)) / H(i, i), lo[i], hi[i]);
            change = std::max(change, std::abs(n - v[i]));
            v[i] = n;
          }
          if (change < 1e-7) break;
        }
        for (int i = 0; i < 7; i++) {
          out[side * 8 + i] = q[i] + h * v[i];
          d_->qpos[m_->jnt_qposadr[joints_[side * 7 + i]]] = out[side * 8 + i];
        }
      }
    out[7] = target.grippers[0];
    out[15] = target.grippers[1];
    return cfg_.limit(out, seed, dt);
  }
};
}  // namespace luxi
