// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <deque>
#include <luxi/core.hpp>
#include <std_msgs/msg/string.hpp>
class Manager : public rclcpp::Node {
  luxi::ArmConfig cfg_;
  luxi::State state_;
  luxi::Command intent_;
  luxi::Quest quest_;
  std::array<double, 16> last_{};
  std::string phase_ = "disabled";
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
      transition_ = luxi::mono();
      life("enable");
      return true;
    }
    if (s == "stop") {
      if (phase_ == "disabled") return true;
      phase_ = "returning";
      transition_ = luxi::mono();
      return true;
    }
    if (s == "disable") {
      life("disable");
      phase_ = "disabled";
      return true;
    }
    return false;
  }

 public:
  Manager()
      : Node("control_manager"), cfg_(luxi::config(declare_parameter<std::string>("config", ""))) {
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
    if (phase_ != "disabled") {
      bool healthy = luxi::valid_state(state_) && luxi::fresh(state_.receive_mono_ns, t);
      if (!healthy) {
        phase_ = "fault";
      }  // Never pretend missing motor feedback is a valid measurement.
      if (phase_ == "homing" || phase_ == "returning") {
        if (healthy) {
          last_ = cfg_.limit(cfg_.home, last_, dt);
          bool arrived = true;
          for (int i = 0; i < 16; i++)
            if (i % 8 != 7 && std::abs(state_.position[i] - cfg_.home[i]) > .05) arrived = false;
          if (arrived && t - transition_ > 2000000000) {
            if (phase_ == "returning") {
              life("disable");
              phase_ = "disabled";
            } else
              phase_ = "aligning";
          }
          if (t - transition_ > 30000000000) phase_ = "fault";
        }
      } else if (phase_ == "aligning" || phase_ == "teleop") {
        if (!luxi::fresh(quest_.receive_mono_ns, t) || !luxi::fresh(intent_.source_mono_ns, t) ||
            quest_.left_tracking == 2 || quest_.right_tracking == 2) {
          phase_ = "hold";
        } else if (phase_ == "aligning") {
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
          if (aligned) phase_ = "teleop";
        } else
          last_ = cfg_.limit(intent_.position, last_, dt);
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
