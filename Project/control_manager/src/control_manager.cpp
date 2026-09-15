// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <deque>
#include <luxi/core.hpp>
#include <std_msgs/msg/string.hpp>
class Manager : public rclcpp::Node {
  luxi::ArmConfig cfg_, home_cfg_;
  double home_tolerance_ = .05;
  double home_joint_speed_limit_ = 0.;
  double input_hold_timeout_ = .5;
  int64_t input_hold_ns_ = 500000000;
  luxi::State state_;
  luxi::Command intent_;
  luxi::Quest quest_;
  std::array<double, 16> last_{};
  std::string phase_ = "disabled";
  std::string reason_ = "disabled";
  bool input_paused_ = false;
  int64_t last_tracking_ = 0, tracking_resume_after_ = 0;
  int64_t previous_ = 0, transition_ = 0;
  bool x_ = false, long_x_ = false;
  int64_t x_start_ = 0;
  std::deque<int64_t> arrivals_;
  rclcpp::Publisher<luxi::Command>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_, status_;
  rclcpp::Subscription<luxi::State>::SharedPtr states_;
  rclcpp::Subscription<luxi::Command>::SharedPtr targets_;
  rclcpp::Subscription<luxi::Quest>::SharedPtr quests_;
  rclcpp::Service<luxi::Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr tick_;
  void life(const std::string& s) {
    std_msgs::msg::String m;
    m.data = s;
    lifecycle_->publish(m);
  }
  bool command(const std::string& s) {
    if (s == "start") {
      if (phase_ != "disabled" || arrivals_.size() < 50 ||
          !luxi::fresh(quest_.receive_mono_ns, luxi::mono()) || quest_.left_tracking == 2 ||
          quest_.right_tracking == 2 || !luxi::valid_state(state_) ||
          !luxi::fresh(state_.receive_mono_ns, luxi::mono()))
        return false;
      last_ = state_.position;
      phase_ = "homing";
      reason_ = "operator_start";
      transition_ = luxi::mono();
      life("enable");
      return true;
    }
    if (s == "stop") {
      if (phase_ == "disabled") return true;
      phase_ = "returning";
      reason_ = "operator_return";
      transition_ = luxi::mono();
      return true;
    }
    if (s == "disable") {
      life("disable");
      phase_ = "disabled";
      reason_ = "operator_disable";
      return true;
    }
    return false;
  }

 public:
  Manager()
      : Node("control_manager"), cfg_(luxi::config(declare_parameter<std::string>("config", ""))),
        home_cfg_(cfg_) {
    home_tolerance_ = declare_parameter<double>("home_tolerance", .05);
    if (!std::isfinite(home_tolerance_) || home_tolerance_ <= 0 || home_tolerance_ > .1)
      throw std::invalid_argument("home_tolerance must be finite and in (0, 0.1] radians");
    home_joint_speed_limit_ = declare_parameter<double>("home_joint_speed_limit", 0.);
    if (!std::isfinite(home_joint_speed_limit_) || home_joint_speed_limit_ < 0.)
      throw std::invalid_argument("home_joint_speed_limit must be finite and nonnegative");
    // A separate, optional cap keeps startup/return slow when following is faster.
    // Zero preserves the configured limits. Grippers keep their configured speed.
    if (home_joint_speed_limit_ > 0.)
      for (int i = 0; i < 16; ++i)
        if (i % 8 != 7)
          home_cfg_.speed[i] = std::min(cfg_.speed[i], home_joint_speed_limit_);
    input_hold_timeout_ = declare_parameter<double>("input_hold_timeout", .5);
    if (!std::isfinite(input_hold_timeout_) || input_hold_timeout_ < .15 || input_hold_timeout_ > 1.)
      throw std::invalid_argument("input_hold_timeout must be finite and in [0.15, 1.0] seconds");
    input_hold_ns_ = static_cast<int64_t>(input_hold_timeout_ * 1e9);
    pub_ = create_publisher<luxi::Command>("/luxi/command", 1);
    lifecycle_ = create_publisher<std_msgs::msg::String>("/luxi/driver_lifecycle", 10);
    status_ = create_publisher<std_msgs::msg::String>("/luxi/control_status",
                                                      rclcpp::QoS(1).transient_local());
    states_ = create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) { state_ = *s; });
    targets_ = create_subscription<luxi::Command>(
        "/luxi/intent", 1, [this](luxi::Command::ConstSharedPtr c) { intent_ = *c; });
    quests_ =
        create_subscription<luxi::Quest>("/luxi/quest", 100, [this](luxi::Quest::ConstSharedPtr q) {
          if (!luxi::fresh(q->receive_mono_ns, luxi::mono())) return;
          if (!luxi::fresh(quest_.receive_mono_ns, q->receive_mono_ns, 500000000)) {
            x_ = false;
            long_x_ = false;
            arrivals_.clear();
          }
          if (q->left_tracking != 2 && q->right_tracking != 2) {
            if (quest_.left_tracking == 2 || quest_.right_tracking == 2)
              tracking_resume_after_ = q->receive_mono_ns;
            last_tracking_ = q->receive_mono_ns;
          }
          quest_ = *q;
          arrivals_.push_back(q->receive_mono_ns);
          while (!arrivals_.empty() && q->receive_mono_ns - arrivals_.front() > 1000000000)
            arrivals_.pop_front();
          if (q->buttons[2] && !x_) {
            command("stop");
            x_start_ = q->receive_mono_ns;
            long_x_ = false;
          }
          if (q->buttons[2] && !long_x_ && q->receive_mono_ns - x_start_ >= 1500000000) {
            long_x_ = command("start");
          }
          x_ = q->buttons[2];
        });
    service_ = create_service<luxi::Service>(
        "/luxi/control", [this](const std::shared_ptr<luxi::Service::Request> req,
                                std::shared_ptr<luxi::Service::Response> res) {
          res->success = command(req->command);
          res->message = phase_;
        });
    tick_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });
  }
  void update() {
    auto t = luxi::mono();
    double dt = previous_ ? std::min(.05, (t - previous_) * 1e-9) : .01;
    previous_ = t;
    input_paused_ = false;
    if (phase_ != "disabled") {
      bool healthy = luxi::valid_state(state_) && luxi::fresh(state_.receive_mono_ns, t);
      if (!healthy) {
        phase_ = "fault";
        reason_ = "feedback_unavailable";
      }  // Never pretend missing motor feedback is a valid measurement.
      if (phase_ == "homing" || phase_ == "returning") {
        if (healthy) {
          last_ = home_cfg_.limit(cfg_.home, last_, dt);
          bool arrived = true;
          for (int i = 0; i < 16; i++)
            if (i % 8 != 7 && std::abs(state_.position[i] - cfg_.home[i]) > home_tolerance_)
              arrived = false;
          if (arrived && t - transition_ > 2000000000) {
            if (phase_ == "returning") {
              life("disable");
              phase_ = "disabled";
              reason_ = "returned";
            } else {
              phase_ = "aligning";
              reason_ = "aligning";
            }
          }
          if (t - transition_ > 30000000000) {
            phase_ = "fault";
            reason_ = "home_timeout";
          }
        }
      } else if (phase_ == "aligning" || phase_ == "teleop") {
        const bool invalid_tracking = quest_.left_tracking == 2 || quest_.right_tracking == 2;
        if (!luxi::fresh(quest_.receive_mono_ns, t)) {
          // Network delivery may be bursty even at a healthy average packet rate.
          // Freeze at 150 ms; only a longer outage latches operator recovery.
          input_paused_ = true;
          reason_ = "quest_gap";
          tracking_resume_after_ = std::max(tracking_resume_after_, quest_.receive_mono_ns + 1);
          if (!luxi::fresh(quest_.receive_mono_ns, t, input_hold_ns_)) {
            phase_ = "hold";
            reason_ = "quest_timeout";
          }
        } else if (invalid_tracking) {
          // Freeze the submitted target immediately. A brief tracking interruption
          // may recover within the existing 150 ms freshness window.
          input_paused_ = true;
          reason_ = "tracking_gap";
          if (!luxi::fresh(last_tracking_, t)) {
            phase_ = "hold";
            reason_ = "tracking_invalid";
          }
        } else if (tracking_resume_after_ && intent_.source_mono_ns < tracking_resume_after_) {
          // Do not follow a cached target before IK has processed the recovered pose.
          input_paused_ = true;
          reason_ = "waiting_for_recovered_ik";
          if (!luxi::fresh(tracking_resume_after_, t, input_hold_ns_)) {
            phase_ = "hold";
            reason_ = "ik_timeout";
          }
        } else if (!luxi::fresh(intent_.source_mono_ns, t)) {
          input_paused_ = true;
          reason_ = "ik_gap";
          if (!luxi::fresh(intent_.source_mono_ns, t, input_hold_ns_)) {
            phase_ = "hold";
            reason_ = "ik_timeout";
          }
        } else if (phase_ == "aligning" && intent_.phase == "ik_unconverged") {
          // A finite, joint-limited IK iterate may still place the hand far
          // from its requested pose. Keep holding home until IK has converged.
          input_paused_ = true;
          reason_ = "waiting_for_ik_solution";
        } else if (phase_ == "aligning") {
          tracking_resume_after_ = 0;
          reason_ = "aligning";
          bool aligned = true;
          auto next = intent_.position;
          for (int s = 0; s < 2; s++) {
            bool trigger = quest_.triggers[s ? 0 : 1] > .92;
            for (int i = 0; i < 7; i++) {
              int k = s * 8 + i;
              if (std::abs(next[k] - state_.position[k]) > .1) aligned = false;
              next[k] =
                  trigger ? last_[k] + std::clamp(next[k] - last_[k], -.2 * dt, .2 * dt) : last_[k];
            }
          }
          last_ = cfg_.limit(next, last_, dt);
          if (aligned) {
            phase_ = "teleop";
            reason_ = "teleop";
          }
        } else {
          tracking_resume_after_ = 0;
          reason_ = "teleop";
          last_ = cfg_.limit(intent_.position, last_, dt);
        }
      }
      if (phase_ != "disabled" && healthy) {
        luxi::Command out;
        out.header.stamp = now();
        out.header.frame_id = "arm_origin";
        out.sequence = intent_.sequence;
        out.source_mono_ns = t;
        out.position = last_;
        out.phase = phase_;
        pub_->publish(out);
      }
    }
    std_msgs::msg::String s;
    s.data = luxi::Json{{"phase", phase_},
                        {"reason", reason_},
                        {"input_paused", input_paused_},
                        {"quest_age_ms", quest_.receive_mono_ns ? (t - quest_.receive_mono_ns) / 1e6 : -1.0},
                        {"intent_age_ms", intent_.source_mono_ns ? (t - intent_.source_mono_ns) / 1e6 : -1.0},
                        {"home_tolerance", home_tolerance_},
                        {"home_joint_speed_limit", home_joint_speed_limit_},
                        {"joint_speed_limits", cfg_.speed},
                        {"input_hold_timeout", input_hold_timeout_},
                        {"vr_fresh", luxi::fresh(quest_.receive_mono_ns, t)},
                        {"tracking_left", quest_.left_tracking},
                        {"tracking_right", quest_.right_tracking}}
                 .dump();
    status_->publish(s);
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Manager>(); });
}
