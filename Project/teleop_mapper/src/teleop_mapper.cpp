// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <luxi/core.hpp>
class Mapper : public rclcpp::Node {
  std::array<luxi::PoseFilter, 2> filters_;
  luxi::Json profile_;
  luxi::ArmConfig config_;
  std::array<double, 2> teleop_open_{};
  std::array<double, 2> wrist_roll_offsets_{};
  int64_t previous_input_ = 0;
  bool heading_only_orientation_reference_ = true;
  Eigen::Vector3d right_position_offset_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond right_orientation_offset_ = Eigen::Quaterniond::Identity();
  rclcpp::Publisher<luxi::Targets>::SharedPtr pub_;
  rclcpp::Subscription<luxi::Quest>::SharedPtr sub_;

 public:
  Mapper()
      : Node("teleop_mapper"),
        profile_(luxi::config(declare_parameter<std::string>("config", ""))), config_(profile_) {
    const auto limits = profile_.value("teleop_gripper_max_open_degrees",
                                      std::vector<double>{-1.0, -1.0});
    if (limits.size() != 2)
      throw std::invalid_argument("teleop_gripper_max_open_degrees needs [right, left]");
    for (int side = 0; side < 2; ++side) {
      const double full = config_.open[side] - config_.closed[side];
      const double degrees = limits[side];
      if (!std::isfinite(degrees) || (degrees != -1.0 &&
          (degrees < 0.0 || degrees > std::abs(full) * 180.0 / M_PI + 1e-8)))
        throw std::invalid_argument("teleop gripper opening must be -1 (full) or within calibrated range");
      teleop_open_[side] = degrees == -1.0 ? config_.open[side] :
          config_.closed[side] + std::copysign(degrees * M_PI / 180.0, full);
    }
    const auto wrist_roll = profile_.value("teleop_wrist_roll_offset_degrees",
                                           std::vector<double>{0.0, 0.0});
    if (wrist_roll.size() != 2)
      throw std::invalid_argument("teleop_wrist_roll_offset_degrees needs [right, left]");
    for (int side = 0; side < 2; ++side) {
      if (!std::isfinite(wrist_roll[side]) || std::abs(wrist_roll[side]) > 180.0)
        throw std::invalid_argument("teleop wrist roll offset must be finite and within +/-180 degrees");
      wrist_roll_offsets_[side] = wrist_roll[side] * M_PI / 180.0;
    }
    heading_only_orientation_reference_ = declare_parameter("heading_only_orientation_reference", true);
    const auto offset = declare_parameter<std::vector<double>>(
        "right_orientation_offset_wxyz", std::vector<double>{1.0, 0.0, 0.0, 0.0});
    if (offset.size() != 4 || !std::all_of(offset.begin(), offset.end(),
        [](double x) { return std::isfinite(x); }))
      throw std::invalid_argument("right orientation offset needs four finite wxyz values");
    right_orientation_offset_ = Eigen::Quaterniond(offset[0], offset[1], offset[2], offset[3]);
    if (right_orientation_offset_.norm() < 1e-6)
      throw std::invalid_argument("right orientation offset quaternion must be nonzero");
    right_orientation_offset_.normalize();
    const auto position_offset = declare_parameter<std::vector<double>>(
        "right_position_offset_xyz", std::vector<double>{0.0, 0.0, 0.0});
    if (position_offset.size() != 3 || !std::all_of(position_offset.begin(), position_offset.end(),
        [](double x) { return std::isfinite(x); }))
      throw std::invalid_argument("right position offset needs three finite xyz values");
    right_position_offset_ = Eigen::Vector3d(position_offset[0], position_offset[1], position_offset[2]);
    if (right_position_offset_.norm() > .3)
      throw std::invalid_argument("right position offset exceeds 0.3 metres");
    const auto cutoff = declare_parameter("pose_filter_cutoff_hz", 2.0);
    for (auto& filter : filters_) filter = luxi::PoseFilter(cutoff);
    pub_ = create_publisher<luxi::Targets>("/luxi/targets", 1);
    sub_ =
        create_subscription<luxi::Quest>("/luxi/quest", 1, [this](luxi::Quest::ConstSharedPtr q) {
          if (!luxi::fresh(q->receive_mono_ns, luxi::mono())) {
            for (auto& f : filters_) f.reset();
            return;
          }
          if (!luxi::fresh(previous_input_, q->receive_mono_ns))
            for (auto& f : filters_) f.reset();
          previous_input_ = q->receive_mono_ns;
          luxi::Targets t;
          t.header = q->header;
          t.header.frame_id = "arm_origin";
          t.sequence = q->sequence;
          t.receive_mono_ns = q->receive_mono_ns;
          t.valid = {q->right_tracking != 2, q->left_tracking != 2};
          for (int s = 0; s < 2; s++) {
            t.grippers[s] =
                teleop_open_[s] + (config_.closed[s] - teleop_open_[s]) * q->triggers[s ? 0 : 1];
            if (!t.valid[s]) {
              filters_[s].reset();
              continue;
            }
            auto mapped = luxi::map_pose(s ? q->left : q->right, q->reference,
                                          heading_only_orientation_reference_);
            // Calibrate right position in arm_origin and right orientation in the tool frame.
            if (s == 0)
              mapped = luxi::pose(luxi::pos(mapped) + right_position_offset_,
                                 luxi::rot(mapped) * right_orientation_offset_);
            // Calibrate neutral wrist roll in the tool frame, whose X axis is J7.
            mapped = luxi::pose(luxi::pos(mapped), luxi::rot(mapped) * Eigen::Quaterniond(
                Eigen::AngleAxisd(wrist_roll_offsets_[s], Eigen::Vector3d::UnitX())));
            auto p = filters_[s].update(mapped, q->receive_mono_ns);
            if (s)
              t.left = p;
            else
              t.right = p;
          }
          pub_->publish(t);
        });
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Mapper>(); });
}
