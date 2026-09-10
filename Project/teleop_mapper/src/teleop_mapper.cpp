// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <luxi/core.hpp>
class Mapper : public rclcpp::Node {
  std::array<luxi::PoseFilter, 2> filters_;
  luxi::ArmConfig config_;
  rclcpp::Publisher<luxi::Targets>::SharedPtr pub_;
  rclcpp::Subscription<luxi::Quest>::SharedPtr sub_;

 public:
  Mapper()
      : Node("teleop_mapper"), config_(luxi::config(declare_parameter<std::string>("config", ""))) {
    pub_ = create_publisher<luxi::Targets>("/luxi/targets", 1);
    sub_ =
        create_subscription<luxi::Quest>("/luxi/quest", 100, [this](luxi::Quest::ConstSharedPtr q) {
          if (!luxi::fresh(q->receive_mono_ns, luxi::mono())) {
            for (auto& f : filters_) f.reset();
            return;
          }
          luxi::Targets t;
          t.header = q->header;
          t.header.frame_id = "arm_origin";
          t.sequence = q->sequence;
          t.receive_mono_ns = q->receive_mono_ns;
          t.valid = {q->right_tracking != 2, q->left_tracking != 2};
          for (int s = 0; s < 2; s++) {
            t.grippers[s] =
                config_.open[s] + (config_.closed[s] - config_.open[s]) * q->triggers[s ? 0 : 1];
            if (!t.valid[s]) {
              filters_[s].reset();
              continue;
            }
            auto p = filters_[s].update(luxi::map_pose(s ? q->left : q->right, q->reference),
                                        q->receive_mono_ns);
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
