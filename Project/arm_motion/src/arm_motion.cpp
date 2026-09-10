// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <luxi/ik.hpp>
class Motion : public rclcpp::Node {
  luxi::ArmConfig cfg_;
  luxi::Ik ik_;
  luxi::State state_;
  luxi::Targets target_;
  rclcpp::Publisher<luxi::Command>::SharedPtr pub_;
  rclcpp::Subscription<luxi::State>::SharedPtr states_;
  rclcpp::Subscription<luxi::Targets>::SharedPtr targets_;
  rclcpp::TimerBase::SharedPtr tick_;

 public:
  Motion()
      : Node("arm_motion"),
        cfg_(luxi::config(declare_parameter<std::string>("config", ""))),
        ik_(declare_parameter<std::string>("model", ""), cfg_) {
    pub_ = create_publisher<luxi::Command>("/luxi/intent", 1);
    states_ = create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) { state_ = *s; });
    targets_ = create_subscription<luxi::Targets>(
        "/luxi/targets", 1, [this](luxi::Targets::ConstSharedPtr t) { target_ = *t; });
    tick_ = create_wall_timer(std::chrono::milliseconds(33), [this] {
      if (!target_.valid[0] || !target_.valid[1] ||
          !luxi::fresh(target_.receive_mono_ns, luxi::mono()) ||
          !luxi::fresh(state_.receive_mono_ns, luxi::mono()) || !luxi::valid_state(state_))
        return;
      try {
        luxi::Command c;
        c.header.stamp = now();
        c.header.frame_id = "arm_origin";
        c.sequence = target_.sequence;
        c.source_mono_ns = target_.receive_mono_ns;
        c.position = ik_.solve(target_, state_.position);
        c.phase = "teleop";
        pub_->publish(c);
      } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "IK: %s", e.what());
      }
    });
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Motion>(); });
}
