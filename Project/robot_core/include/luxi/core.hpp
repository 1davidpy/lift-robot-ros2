// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <luxi_interfaces/msg/arm_command.hpp>
#include <luxi_interfaces/msg/arm_state.hpp>
#include <luxi_interfaces/msg/arm_targets.hpp>
#include <luxi_interfaces/msg/camera_frame.hpp>
#include <luxi_interfaces/msg/quest_input.hpp>
#include <luxi_interfaces/srv/command.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
namespace luxi {
using Json = nlohmann::json;
using Quest = luxi_interfaces::msg::QuestInput;
using Targets = luxi_interfaces::msg::ArmTargets;
using Command = luxi_interfaces::msg::ArmCommand;
using State = luxi_interfaces::msg::ArmState;
using Frame = luxi_interfaces::msg::CameraFrame;
using Service = luxi_interfaces::srv::Command;
inline int64_t mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
inline bool fresh(int64_t t, int64_t now, int64_t ttl = 150000000) {
  return t > 0 && now >= t && now - t <= ttl;
}
inline Json config(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open config: " + path);
  Json j;
  in >> j;
  return j;
}
inline Eigen::Vector3d pos(const geometry_msgs::msg::Pose& p) {
  return {p.position.x, p.position.y, p.position.z};
}
inline Eigen::Quaterniond rot(const geometry_msgs::msg::Pose& p) {
  return {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
}
inline geometry_msgs::msg::Pose pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Pose out;
  out.position.x = p.x();
  out.position.y = p.y();
  out.position.z = p.z();
  out.orientation.w = q.w();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  return out;
}
inline double number(const Json& j, const char* key) {
  double v = j.at(key).get<double>();
  if (!std::isfinite(v)) throw std::invalid_argument("nonfinite field");
  return v;
}
inline geometry_msgs::msg::Pose parse_pose(const Json& j) {
  Eigen::Vector3d p(number(j, "x"), number(j, "y"), number(j, "z"));
  Eigen::Quaterniond q(number(j, "qw"), number(j, "qx"), number(j, "qy"), number(j, "qz"));
  if (q.norm() < 1e-8 || p.norm() > 100) throw std::invalid_argument("invalid pose");
  return pose(p, q.normalized());
}
inline Quest parse_quest(const std::string& raw) {
  auto j = Json::parse(raw);
  Quest q;
  q.sender_time = number(j, "t");
  q.reference.orientation.w = 1;
  if (j.contains("rf")) q.reference = parse_pose(j.at("rf"));
  auto flag = [&](const char* key) {
    int v = j.value(key, 0);
    if (v < 0 || v > 2) throw std::invalid_argument("tracking range");
    return static_cast<uint8_t>(v);
  };
  q.tracking = flag("v");
  q.left_tracking = flag("vl");
  q.right_tracking = flag("vr");
  if (q.tracking > 2 || q.left_tracking > 2 || q.right_tracking > 2)
    throw std::invalid_argument("tracking range");
  auto controller = [&j](const char* key, geometry_msgs::msg::Pose& p, uint8_t& tracking) {
    const auto it = j.find(key);
    if (it == j.end() || it->is_null()) {
      // The APK keeps sending timestamps/buttons when a pose is unavailable.
      // Preserve reception and stop controls, but never expose a usable target.
      tracking = 2;
      p.orientation.w = 1;
      return;
    }
    p = parse_pose(*it);
  };
  controller("lc", q.left, q.left_tracking);
  controller("rc", q.right, q.right_tracking);
  for (int i = 0; i < 2; i++) {
    q.triggers[i] = number(j, i ? "rt" : "lt");
    q.grips[i] = j.value(i ? "rg" : "lg", 0.0);
    if (!std::isfinite(q.grips[i]) || q.triggers[i] < 0 || q.triggers[i] > 1 || q.grips[i] < 0 ||
        q.grips[i] > 1)
      throw std::invalid_argument("trigger range");
  }
  const char* keys[] = {"lsx", "lsy", "rsx", "rsy"};
  const char* buttons[] = {"a", "b", "x", "y"};
  for (int i = 0; i < 4; i++) {
    q.sticks[i] = j.value(keys[i], 0.0);
    if (!std::isfinite(q.sticks[i]) || std::abs(q.sticks[i]) > 1)
      throw std::invalid_argument("stick range");
    q.buttons[i] = j.value(buttons[i], false);
  }
  return q;
}
inline geometry_msgs::msg::Pose map_pose(const geometry_msgs::msg::Pose& p,
                                         const geometry_msgs::msg::Pose& reference,
                                         bool heading_only_orientation_reference = false) {
  auto flip = [](const auto& v) {
    auto x = pos(v);
    x.z() = -x.z();
    auto q = rot(v);
    q.x() = -q.x();
    q.y() = -q.y();
    return std::make_pair(x, q);
  };
  auto [x, q] = flip(p);
  auto [r, rq] = flip(reference);
  Eigen::Matrix3d m;
  m << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  auto orientation_reference = rq;
  if (heading_only_orientation_reference) {
    // Keep headset pitch/roll out of wrist commands. Position retains its existing frame.
    const auto forward = rq * Eigen::Vector3d::UnitZ();
    const auto right = rq * Eigen::Vector3d::UnitX();
    const double yaw = std::hypot(forward.x(), forward.z()) > 1e-6
                           ? std::atan2(forward.x(), forward.z())
                           : std::atan2(-right.z(), right.x());
    orientation_reference = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
  }
  return pose(m * (rq.conjugate() * (x - r)) + Eigen::Vector3d(-.085, 0, -.14),
              Eigen::Quaterniond(m) * orientation_reference.conjugate() * q *
                  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));
}
class PoseFilter {
  bool ready_ = false;
  int64_t t_ = 0;
  double min_cutoff_hz_;
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero(), velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
  static double alpha(double hz, double dt) { return 1.0 / (1.0 + 1.0 / (2 * M_PI * hz * dt)); }

 public:
  explicit PoseFilter(double min_cutoff_hz = 2.0) : min_cutoff_hz_(min_cutoff_hz) {
    if (!std::isfinite(min_cutoff_hz_) || min_cutoff_hz_ <= 0)
      throw std::invalid_argument("pose filter cutoff must be finite and positive");
  }
  void reset() { ready_ = false; velocity_.setZero(); }
  geometry_msgs::msg::Pose update(const geometry_msgs::msg::Pose& p, int64_t t) {
    auto x = pos(p);
    auto q = rot(p);
    if (!ready_) {
      ready_ = true;
      p_ = x;
      q_ = q;
      t_ = t;
      return p;
    }
    double dt = std::clamp((t - t_) * 1e-9, 1e-4, .1);
    t_ = t;
    double ad = alpha(1.5, dt);
    velocity_ = ad * (x - p_) / dt + (1 - ad) * velocity_;
    double a = alpha(min_cutoff_hz_ + .04 * velocity_.norm(), dt);
    p_ += a * (x - p_);
    q_ = q_.slerp(a, q);
    return pose(p_, q_);
  }
};
struct ArmConfig {
  std::array<double, 16> home{}, lo{}, hi{}, speed{}, kp{}, kd{}, offset{};
  std::array<double, 2> open{{-1, 1}}, closed{{0, 0}};
  explicit ArmConfig(const Json& j) {
    for (int side = 0; side < 2; side++) {
      auto a = j.at(side ? "left" : "right");
      for (int i = 0; i < 8; i++) {
        int k = side * 8 + i;
        home[k] = a.at("home").at(i);
        lo[k] = a.at("limits").at(i).at(0);
        hi[k] = a.at("limits").at(i).at(1);
        speed[k] = j.at("velocity").at(i);
        kp[k] = j.at("kp").at(i);
        kd[k] = j.at("kd").at(i);
        offset[k] = a.at("offsets").at(i);
        if (!std::isfinite(home[k]) || !std::isfinite(lo[k]) || !std::isfinite(hi[k]) ||
            !std::isfinite(speed[k]) || !std::isfinite(kp[k]) || !std::isfinite(kd[k]) ||
            !std::isfinite(offset[k]) || lo[k] >= hi[k] || speed[k] <= 0 || kp[k] < 0 || kd[k] < 0)
          throw std::invalid_argument("arm config");
      }
      open[side] = a.at("gripper_open");
      closed[side] = a.at("gripper_closed");
    }
  }
  std::array<double, 16> limit(const std::array<double, 16>& target,
                               const std::array<double, 16>& previous, double dt) const {
    auto out = previous;
    dt = std::clamp(dt, 0.0, .05);
    for (int i = 0; i < 16; i++) {
      if (!std::isfinite(target[i])) throw std::invalid_argument("nonfinite command");
      // Apply physical joint limits to the gripper as in the original JointPosChecker.
      double low = lo[i], high = hi[i];
      out[i] = previous[i] + std::clamp(std::clamp(target[i], low, high) - previous[i],
                                        -speed[i] * dt, speed[i] * dt);
    }
    return out;
  }
};
inline bool valid_state(const State& s) {
  return std::all_of(s.valid.begin(), s.valid.end(), [](bool x) { return x; }) &&
         std::all_of(s.position.begin(), s.position.end(),
                     [](double x) { return std::isfinite(x); });
}
inline int node_main(int argc, char** argv, const std::function<rclcpp::Node::SharedPtr()>& make) {
  rclcpp::init(argc, argv);
  try {
    auto node = make();
    auto shutdown = node->create_subscription<std_msgs::msg::String>(
        "/luxi/shutdown", 10, [](std_msgs::msg::String::ConstSharedPtr) { rclcpp::shutdown(); });
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
}  // namespace luxi
