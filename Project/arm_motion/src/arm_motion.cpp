// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <luxi/ik.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
class Motion : public rclcpp::Node {
  luxi::ArmConfig cfg_;
  luxi::Ik ik_;
  luxi::State state_;
  luxi::Targets target_;
  std::array<double, 16> reference_{};
  bool reference_ready_ = false;
  bool network_recovering_ = false;
  std::string control_phase_;
  bool teleop_owner_ = false;
  int64_t control_time_ = 0;
  std::array<geometry_msgs::msg::Quaternion, 2> edge_orientation_;
  bool edge_locked_ = false;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr edge_status_;
  rclcpp::Service<luxi::Service>::SharedPtr edge_service_;

  void set_edge_lock(bool locked) {
    edge_locked_ = locked;
    std_msgs::msg::Bool status;
    status.data = locked;
    edge_status_->publish(status);
  }

  void edge_command(const std::shared_ptr<luxi::Service::Request> req,
                    std::shared_ptr<luxi::Service::Response> res) {
    const auto now = luxi::mono();
    const bool lock = req->command == "lock";
    if ((!lock && req->command != "unlock") || control_phase_ != "teleop" ||
        !teleop_owner_ || network_recovering_ || !luxi::fresh(control_time_, now, 500000000) ||
        !luxi::fresh(target_.receive_mono_ns, now) || !target_.valid[0] || !target_.valid[1] ||
        !state_.enabled || !luxi::fresh(state_.receive_mono_ns, now) || !luxi::valid_state(state_)) {
      res->message = "请在双臂遥操就绪、跟踪和关节反馈有效时操作";
      return;
    }
    if (lock != edge_locked_) {
      if (lock) {
        ik_.sync(state_.position);
        edge_orientation_ = {ik_.fk(0).orientation, ik_.fk(1).orientation};
      } else {
        // Releasing while the operator's wrists are turned would step the IK goal.
        for (int side = 0; side < 2; ++side) {
          const auto& hand = side ? target_.left : target_.right;
          auto locked = hand;
          locked.orientation = edge_orientation_[side];
          if (luxi::rot(hand).angularDistance(luxi::rot(locked)) > 5.0 * M_PI / 180.0) {
            res->message = "请先将两个手柄转回锁定时的朝向，再解除锁定（允许偏差 5°）";
            return;
          }
        }
      }
      set_edge_lock(lock);
    }
    res->success = true;
    res->message = lock ? "双爪朝向已锁定" : "已恢复手柄转向";
  }
  rclcpp::Publisher<luxi::Command>::SharedPtr pub_;
  rclcpp::Subscription<luxi::State>::SharedPtr states_;
  rclcpp::Subscription<luxi::Targets>::SharedPtr targets_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_;
  rclcpp::TimerBase::SharedPtr tick_;

 public:
  Motion()
      : Node("arm_motion"),
        cfg_(luxi::config(declare_parameter<std::string>("config", ""))),
        ik_(declare_parameter<std::string>("model", ""), cfg_) {
    edge_status_ = create_publisher<std_msgs::msg::Bool>(
        "/luxi/edge_lock/status", rclcpp::QoS(1).transient_local());
    set_edge_lock(false);
    edge_service_ = create_service<luxi::Service>(
        "/luxi/edge_lock", std::bind(&Motion::edge_command, this,
                                    std::placeholders::_1, std::placeholders::_2));
    pub_ = create_publisher<luxi::Command>("/luxi/intent", 1);
    states_ = create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) { state_ = *s; });
    targets_ = create_subscription<luxi::Targets>(
        "/luxi/targets", 1, [this](luxi::Targets::ConstSharedPtr t) { target_ = *t; });
    control_ = create_subscription<std_msgs::msg::String>(
        "/luxi/control_status", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          auto status = luxi::Json::parse(message->data, nullptr, false);
          if (!status.is_object() || !status.contains("phase") || !status["phase"].is_string())
            return;
          const auto phase = status["phase"].get<std::string>();
          control_time_ = luxi::mono();
          teleop_owner_ = status.value("control_source", "") == std::string("teleop");
          if (edge_locked_ && (phase == "disabled" || phase == "homing" || !teleop_owner_))
            set_edge_lock(false);
          if (phase == "homing" && control_phase_ != "homing") {
            // A new session must not inherit a solver branch accumulated while
            // the headset was being put down or the previous session was stopped.
            reference_ = cfg_.home;
            for (int i = 0; i < 16; ++i)
              reference_[i] = std::clamp(reference_[i], cfg_.lo[i], cfg_.hi[i]);
            reference_ready_ = true;
          } else if (phase == "disabled") {
            reference_ready_ = false;
          }
          network_recovering_ = status.value("network_recovering", false);
          control_phase_ = phase;
        });
    tick_ = create_wall_timer(std::chrono::milliseconds(33), [this] {
      // Standalone IK tools may omit control_status. A managed session only
      // integrates while preparing or executing arm teleoperation.
      if (!control_phase_.empty() && control_phase_ != "homing" &&
          control_phase_ != "aligning" && control_phase_ != "teleop")
        return;
      if (!target_.valid[0] || !target_.valid[1] ||
          !luxi::fresh(target_.receive_mono_ns, luxi::mono()) ||
          !luxi::fresh(state_.receive_mono_ns, luxi::mono()) || !luxi::valid_state(state_))
        return;
      try {
        // Integrate the commanded IK configuration as in the original mink node.
        // Re-seeding every step from measured feedback loses progress under load.
        auto seed = reference_ready_ ? reference_ : state_.position;
        luxi::Command c;
        c.header.stamp = now();
        c.header.frame_id = "arm_origin";
        c.sequence = target_.sequence;
        c.source_mono_ns = target_.receive_mono_ns;
        // During preparation, settle the virtual IK reference ahead of the
        // physical arm. The control manager still limits alignment motion.
        const bool preparing = control_phase_ == "homing" || control_phase_ == "aligning" || network_recovering_;
        const int iterations = preparing ? 8 : 1;
        const auto deadline = luxi::mono() + 8000000;
        auto goal = target_;
        if (edge_locked_) {
          goal.right.orientation = edge_orientation_[0];
          goal.left.orientation = edge_orientation_[1];
        }
        bool converged = false;
        for (int iteration = 0; iteration < iterations; ++iteration) {
          c.position = ik_.solve(goal, seed);
          ik_.sync(c.position);
          converged = true;
          for (int side = 0; side < 2; ++side) {
            const auto actual = ik_.fk(side);
            const auto& desired = side ? goal.left : goal.right;
            if ((luxi::pos(actual) - luxi::pos(desired)).norm() > .02 ||
                luxi::rot(actual).angularDistance(luxi::rot(desired)) > .25)
              converged = false;
          }
          if (converged || luxi::mono() >= deadline) break;
          seed = c.position;
        }
        reference_ = c.position;
        reference_ready_ = true;
        c.phase = converged ? "teleop" : "ik_unconverged";
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
