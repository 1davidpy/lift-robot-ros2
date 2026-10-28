// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <deque>
#include <luxi/core.hpp>
#include <luxi/coordinated_control.hpp>
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
  std::array<double, 16> return_from_{};
  static constexpr int return_steps_ = 100; // Legacy stop.moves: 50 Hz for 2 s.
  static constexpr int64_t return_period_ns_ = 20000000;
  int return_step_ = 0;
  int64_t return_next_ns_ = 0;
  std::string phase_ = "disabled";
  std::string reason_ = "disabled";
  bool input_paused_ = false;
  bool shared_input_paused_ = false;
  int64_t last_tracking_ = 0, tracking_resume_after_ = 0;
  int64_t previous_ = 0, transition_ = 0;
  bool x_ = false, long_x_ = false;
  int64_t x_start_ = 0;
  bool shared_ = false, menu_ = false;
  bool x_released_ = false, x_enable_press_ = false, b_ = false, rest_ready_ = false;
  bool torque_requested_ = false;
  bool manual_pose_blocked_ = false;
  std::string manual_pose_phase_ = "inactive", manual_pose_generation_;
  std::array<double, 16> manual_pose_target_{};
  bool manual_pose_fault_ = false, startup_owned_ = false, return_manual_fault_ = false;
  std::string manual_pose_reason_, return_manual_generation_;
  int64_t return_manual_request_ns_ = 0;
  std::string start_manual_generation_;
  std::array<double, 16> start_manual_target_{};
  int64_t start_manual_request_ns_ = 0;
  bool start_manual_fault_ = false;
  int64_t manual_pose_status_ns_ = 0;
  std::string control_generation_, return_control_generation_;
  std::array<double, 16> support_target_{};
  bool support_target_valid_ = false;
  std::array<double, 16> supported_pose(int64_t t) const {
    return state_.enabled && support_target_valid_ && luxi::fresh(manual_pose_status_ns_, t, 500000000)
        ? support_target_ : state_.position;
  }
  std::string system_phase_ = "disabled", epoch_;
  const std::string owner_ = std::to_string(luxi::mono());
  uint64_t epoch_count_ = 0;
  int64_t system_since_ = 0;
  luxi::Json base_status_, lift_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr coordinated_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr base_sub_, lift_sub_, recorder_sub_, startup_sub_;
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
    // A hand-posed hold accepts only this generation's explicit enable. Plain
    // queued enables from before the edit must not release the captured pose.
    if (s == "disable") {
      const auto& generation = return_control_generation_.empty() ? control_generation_ : return_control_generation_;
      // Older drivers/fixtures have no generation. New drivers reject the
      // unscoped fallback rather than unloading another owner's hold.
      m.data = generation.empty() ? "disable" : "disable:" + generation;
    } else m.data = s == "enable" && manual_pose_phase_ == "holding"
        ? s + ":" + manual_pose_generation_ : s;
    lifecycle_->publish(m);
  }
  bool neutral_input(int64_t t) const {
    if (!luxi::fresh(quest_.receive_mono_ns, t) || quest_.tracking == 2 ||
        quest_.left_tracking == 2 || quest_.right_tracking == 2 || menu_) return false;
    for (auto v : quest_.sticks) if (!std::isfinite(v) || std::abs(v) >= .16) return false;
    for (auto v : quest_.grips) if (!std::isfinite(v) || v < 0 || v >= .35) return false;
    return !quest_.buttons[1];
  }
  bool arm_feedback(int64_t t) const {
    return luxi::valid_state(state_) && luxi::fresh(state_.receive_mono_ns, t);
  }
  void clear_manual_start() {
    start_manual_generation_.clear(); start_manual_request_ns_ = 0;
  }
  bool manual_start_ready(int64_t t) const {
    return manual_pose_phase_ == "holding" && !manual_pose_generation_.empty() && !startup_owned_ &&
        (start_manual_generation_.empty() || start_manual_generation_ == manual_pose_generation_) &&
        luxi::fresh(manual_pose_status_ns_, t, 500000000) && state_.enabled && arm_feedback(t);
  }
  void abort_manual_start(const std::string& why) {
    clear_manual_start();
    phase_ = "disabled"; enter("stopped"); reason_ = why;
    if (arm_feedback(luxi::mono())) last_ = state_.position;
    broadcast(); // The current driver owner retains torque; no unowned target is sent.
  }
  bool reserve_manual_start(int64_t t) {
    if (!manual_start_ready(t)) {
      reason_ = "manual_pose_start_requires_fresh_hold_status"; return false;
    }
    start_manual_generation_ = manual_pose_generation_;
    start_manual_target_ = manual_pose_target_; start_manual_fault_ = manual_pose_fault_;
    start_manual_request_ns_ = 0;
    phase_ = "disabled";
    return true;
  }
  bool request_manual_start(int64_t t) {
    if (!manual_start_ready(t)) {
      abort_manual_start("manual_pose_changed_before_start"); return false;
    }
    start_manual_request_ns_ = luxi::mono();
    reason_ = "waiting_for_manual_pose_handover";
    life("enable");
    return true;
  }
  void update_manual_start(int64_t t) {
    if (start_manual_generation_.empty()) return;
    if (manual_pose_generation_ != start_manual_generation_ || startup_owned_ ||
        (manual_pose_phase_ != "holding" &&
         !(manual_pose_phase_ == "inactive" && manual_pose_reason_ == "lifecycle_handover"))) {
      abort_manual_start("manual_pose_start_taken_over"); return;
    }
    if (!start_manual_request_ns_) {
      if (!manual_start_ready(t)) abort_manual_start("manual_pose_changed_before_start");
      return; // Shared base/lift preparation has not requested arm handover yet.
    }
    const bool acknowledged = manual_pose_status_ns_ > start_manual_request_ns_ &&
        luxi::fresh(manual_pose_status_ns_, t, 500000000) && manual_pose_phase_ == "inactive" &&
        manual_pose_reason_ == "lifecycle_handover" && state_.enabled && arm_feedback(t) &&
        state_.receive_mono_ns > start_manual_request_ns_;
    if (!acknowledged) {
      if (t - start_manual_request_ns_ > 1000000000)
        abort_manual_start("manual_pose_start_handover_timeout");
      return;
    }
    // Keep the support target for a healthy hold. A latched fault instead
    // starts from post-handover feedback, matching the adapter's re-anchoring.
    last_ = start_manual_fault_ ? state_.position : start_manual_target_;
    clear_manual_start();
    phase_ = "homing"; transition_ = t; reason_ = "homing";
  }
  bool parking_in_progress() const {
    return system_phase_ == "stopping_base" || system_phase_ == "returning_arms" ||
           system_phase_ == "disabling";
  }
  bool matching(const luxi::Json& j, int64_t t) const {
    return luxi::coordinated_status_fresh(j, t) && j.value("shared_enable", false) &&
           j.value("shared_epoch", "") == epoch_ && !j.value("shared_failed", true);
  }
  bool device_feedback(int64_t t) const {
    try {
      const auto base_time = std::stoll(base_status_.at("feedback_mono_ns").get<std::string>());
      return base_status_.value("feedback_fresh", false) && luxi::fresh(base_time, t, 250000000) &&
             luxi::coordinated_lift_feedback(lift_status_, t);
    } catch (const std::exception&) { return false; }
  }
  bool begin_return(int64_t t) {
    // Python SingleArmDriver.stop(): smooth_move(stop pose, 50 Hz, 2 s),
    // then disable_all(). Completion is the command sequence, not measured
    // home convergence. The existing arm driver still enforces joint limits
    // and its hardware velocity limits on every submitted waypoint.
    if (!return_manual_generation_.empty()) {
      if (manual_pose_phase_ != "holding" || manual_pose_generation_ != return_manual_generation_ ||
          startup_owned_ || !luxi::fresh(manual_pose_status_ns_, t, 500000000)) {
        abort_manual_return("manual_pose_changed_before_return"); return false;
      }
      return_manual_fault_ = manual_pose_fault_;
      last_ = manual_pose_fault_ ? state_.position : manual_pose_target_;
      return_manual_request_ns_ = luxi::mono();
      life("enable");
    }
    return_control_generation_ = control_generation_;
    return_from_ = last_;
    return_step_ = 0;
    return_next_ns_ = t;
    rest_ready_ = false;
    phase_ = "returning";
    transition_ = t;
    return true;
  }
  void advance_return(int64_t t) {
    // Do not count a return that the hand-pose owner has not accepted. Otherwise
    // a failed handover could finish an empty sequence and remove torque in place.
    if (!return_manual_generation_.empty()) {
      const bool acknowledged = manual_pose_status_ns_ > return_manual_request_ns_ &&
          luxi::fresh(manual_pose_status_ns_, t, 500000000) &&
          manual_pose_generation_ == return_manual_generation_ && manual_pose_phase_ == "inactive" &&
          manual_pose_reason_ == "lifecycle_handover" && !startup_owned_ && state_.enabled && arm_feedback(t);
      if (!acknowledged) {
        if (manual_pose_generation_ != return_manual_generation_ ||
            (manual_pose_status_ns_ > return_manual_request_ns_ && startup_owned_))
          abort_manual_return("manual_pose_return_taken_over");
        else if (t - transition_ > 1000000000)
          abort_manual_return("manual_pose_handover_timeout");
        return;
      }
      if (return_manual_fault_) last_ = return_from_ = state_.position;
      return_manual_generation_.clear(); return_manual_request_ns_ = 0;
      return_control_generation_ = control_generation_;
      return_next_ns_ = t;
    }
    if (t < return_next_ns_) return;
    if (return_step_ < return_steps_) {
      const double fraction = double(return_step_) / (return_steps_ - 1);
      for (int i = 0; i < 16; ++i)
        last_[i] = std::clamp(return_from_[i] + fraction * (cfg_.home[i] - return_from_[i]),
                              cfg_.lo[i], cfg_.hi[i]);
      ++return_step_;
      // No catch-up jump or skipped waypoint after a delayed callback. Like
      // the Python sleep(1/hz), scheduling delays extend the nominal duration.
      return_next_ns_ = t + return_period_ns_;
      return;
    }
    // The final waypoint has been published for at least one 50 Hz interval.
    if (shared_) {
      rest_ready_ = true;
      phase_ = "hold";
      reason_ = "legacy_return_sequence_complete";
    } else {
      life("disable");
      phase_ = "disabled";
      reason_ = "returned";
    }
  }
  std::string directive() const {
    if (system_phase_ == "preparing" || system_phase_ == "starting") return "prepare";
    if (system_phase_ == "arming" || system_phase_ == "active") return "active";
    if (system_phase_ == "stopping_base") return "stop";
    if (system_phase_ == "returning_arms" || system_phase_ == "disabling") return "park";
    return system_phase_ == "disabled" ? "disabled" : "halt";
  }
  void broadcast() {
    if (!shared_) return;
    std_msgs::msg::String m;
    m.data = luxi::Json{{"epoch", epoch_}, {"phase", directive()},
                         {"sent_mono_ns", std::to_string(luxi::mono())}}.dump();
    coordinated_->publish(m);
  }
  void enter(const std::string& s) { system_phase_ = s; system_since_ = luxi::mono(); }
  void suspend_for_manual_pose() {
    clear_manual_start();
    phase_ = "disabled";
    enter("stopped");
    reason_ = "manual_pose_in_progress";
    torque_requested_ = rest_ready_ = false;
    input_paused_ = shared_input_paused_ = true;
    intent_ = luxi::Command{};
    if (arm_feedback(luxi::mono())) last_ = state_.position;
    return_step_ = 0; return_next_ns_ = 0;
    return_manual_generation_.clear(); return_manual_request_ns_ = 0;
    x_released_ = x_enable_press_ = false;
    long_x_ = true;
    broadcast();
  }
  void abort_manual_return(const std::string& why) {
    return_manual_generation_.clear(); return_manual_request_ns_ = 0;
    phase_ = state_.enabled ? "hold" : "disabled";
    enter("stopped"); reason_ = why;
    if (arm_feedback(luxi::mono())) last_ = state_.position;
    broadcast();
  }
  void hold_all(const std::string& why) {
    const bool waiting_for_owner = !start_manual_generation_.empty();
    clear_manual_start();
    shared_input_paused_ = false;
    if (arm_feedback(luxi::mono()) && state_.enabled) last_ = supported_pose(luxi::mono());
    phase_ = !waiting_for_owner && (state_.enabled || torque_requested_) ? "hold" : "disabled";
    enter("stopped"); reason_ = why; rest_ready_ = false;
    broadcast(); // No return motion and no arm torque removal on this path.
  }
  bool shared_command(const std::string& s) {
    const auto t = luxi::mono();
    if (s == "disable" || s == "halt" || s == "hold") { hold_all("operator_stop_all"); return true; }
    if (s == "start") {
      if ((system_phase_ != "disabled" && system_phase_ != "stopped") ||
          !neutral_input(t) || !arm_feedback(t) || arrivals_.size() < 50 ||
          !luxi::coordinated_status_fresh(base_status_, t) ||
          !luxi::coordinated_status_fresh(lift_status_, t) ||
          !base_status_.value("shared_enable", false) || !lift_status_.value("shared_enable", false) ||
          base_status_.value("receive_only", true) || !lift_status_.value("allow_motion", false) ||
          !base_status_.value("feedback_fresh", false) || !lift_status_.value("feedback_healthy", false)) {
        reason_ = "enable_rejected_need_all_devices_and_neutral_input"; return false;
      }
      if (manual_pose_phase_ == "holding" && !reserve_manual_start(t)) return false;
      epoch_ = owner_ + ":" + std::to_string(++epoch_count_);
      if (!start_manual_generation_.empty()) phase_ = "disabled";
      else if (state_.enabled) { last_ = state_.position; phase_ = "hold"; }
      else phase_ = "disabled";
      rest_ready_ = false; enter("preparing"); reason_ = "checking_base_and_lift"; broadcast(); return true;
    }
    if (s == "stop" || s == "park") {
      if (system_phase_ == "stopping_base" ||
          system_phase_ == "returning_arms" || system_phase_ == "disabling") return true;
      if (system_phase_ == "disabled" && !state_.enabled && !torque_requested_) return true;
      hold_all("parking_requested");
      if (!state_.enabled || !arm_feedback(t)) { reason_ = "parking_requires_enabled_arms_and_feedback"; return false; }
      return_manual_generation_ = manual_pose_phase_ == "holding" ? manual_pose_generation_ : std::string();
      return_manual_request_ns_ = 0;
      epoch_ = owner_ + ":" + std::to_string(++epoch_count_);
      enter("stopping_base"); broadcast(); return true;
    }
    return false;
  }
  void update_shared(int64_t t) {
    shared_input_paused_ = false;
    if (!shared_) return;
    if (system_phase_ == "disabled" && state_.enabled) {
      torque_requested_ = true; hold_all("arms_enabled_after_controller_restart");
    }
    if (system_phase_ == "disabled" || system_phase_ == "stopped") return;
    if (!arm_feedback(t)) { hold_all("arm_feedback_lost"); return; }
    const bool parking = parking_in_progress();
    // Once the operator requests the fixed legacy return, headset tracking
    // is no longer a trajectory input. B/menu and device faults still halt it.
    if (menu_ || quest_.buttons[1] || (!parking &&
        (!luxi::fresh(quest_.receive_mono_ns, t, input_hold_ns_) || quest_.tracking == 2 ||
         quest_.left_tracking == 2 || quest_.right_tracking == 2))) {
      hold_all("quest_invalid_or_failure_menu"); return;
    }
    // Freeze at 150 ms even while waiting for an adapter to acknowledge a
    // phase change. Device faults below still override the short-gap pause.
    shared_input_paused_ = !parking && !luxi::fresh(quest_.receive_mono_ns, t);
    if (!luxi::coordinated_status_fresh(base_status_, t) || !luxi::coordinated_status_fresh(lift_status_, t)) {
      hold_all("base_or_lift_status_lost"); return;
    }
    const bool base = matching(base_status_, t), lift = matching(lift_status_, t);
    if ((base_status_.value("shared_epoch", "") == epoch_ && base_status_.value("shared_failed", true)) ||
        (lift_status_.value("shared_epoch", "") == epoch_ && lift_status_.value("shared_failed", true))) {
      hold_all("device_stopped_or_rejected_operation"); return;
    }
    if ((!base || !lift) && t - system_since_ > 1000000000) {
      hold_all("device_coordination_rejected"); return;
    }
    if (!base || !lift) return;
    if (!device_feedback(t)) {
      hold_all("base_or_lift_feedback_lost"); return;
    }
    if (system_phase_ == "preparing") {
      if (shared_input_paused_) return;
      if (!neutral_input(t)) { hold_all("release_grips_and_center_sticks_before_enable"); return; }
      if (base_status_.value("shared_ready", false) && lift_status_.value("shared_ready", false)) {
        if (!start_manual_generation_.empty()) {
          if (!request_manual_start(t)) return;
          torque_requested_ = true; enter("starting");
        } else {
          last_ = state_.position; phase_ = "homing"; transition_ = t;
          torque_requested_ = true; life("enable"); enter("starting"); reason_ = "homing";
        }
      } else if (t - system_since_ > 5000000000LL) hold_all("device_prepare_timeout");
    } else if (system_phase_ == "starting") {
      if (!base_status_.value("stationary", false) || !lift_status_.value("shared_ready", false)) {
        hold_all("base_or_lift_moved_during_arm_preparation"); return;
      }
      if (!state_.enabled && t - system_since_ > 1000000000) { hold_all("arm_enable_not_confirmed"); return; }
      if (phase_ == "hold" || phase_ == "fault") { hold_all(reason_); return; }
      if (phase_ == "teleop" && !shared_input_paused_) {
        if (lift_status_.value("input_recovery", false)) return;
        if (!neutral_input(t)) { hold_all("neutral_controls_required_after_alignment"); return; }
        enter("arming"); reason_ = "enabling_base_and_lift";
      }
    } else if (system_phase_ == "arming") {
      if (shared_input_paused_) return;
      if (base_status_.value("enabled", false) && lift_status_.value("enabled", false) &&
          base_status_.value("shared_phase", "") == "active" && lift_status_.value("shared_phase", "") == "active") {
        enter("active"); reason_ = "teleop";
      } else if (t - system_since_ > 1000000000) hold_all("shared_enable_not_confirmed");
    } else if (system_phase_ == "active") {
      if (!state_.enabled || phase_ != "teleop" || !base_status_.value("enabled", false) ||
          !lift_status_.value("enabled", false)) hold_all("subsystem_stopped");
    } else {
      if (!state_.enabled && system_phase_ != "disabling") { hold_all("arm_hold_lost"); return; }
      if (system_phase_ == "stopping_base") {
        if (!base_status_.value("enabled", true) && base_status_.value("stationary", false) &&
            lift_status_.value("stationary", false) && !lift_status_.value("session_present", true)) {
          // Operator-requested legacy arm parking: keep the lift at its
          // stopped height. Never issue a lift HOME/MOVE as part of disable.
          if (begin_return(t)) {
            enter("returning_arms"); reason_ = "legacy_timed_return_lift_stopped";
          }
        } else if (t - system_since_ > 10000000000LL) hold_all("stop_confirmation_timeout");
      } else {
        if (base_status_.value("enabled", true) || !base_status_.value("stationary", false)) {
          hold_all("base_not_stopped_during_parking"); return;
        }
        if (!lift_status_.value("stationary", false) || lift_status_.value("session_present", true)) {
          hold_all("lift_not_stopped_during_arm_return");
        } else if (system_phase_ == "returning_arms") {
          if (phase_ == "fault") { hold_all(reason_); return; }
          if (rest_ready_) {
            life("disable"); enter("disabling"); reason_ = "waiting_for_arm_disable_feedback";
          }
        } else if (system_phase_ == "disabling") {
          if (!state_.enabled) {
            torque_requested_ = false; phase_ = "disabled"; enter("disabled"); reason_ = "parked_and_disabled";
          } else if (t - system_since_ > 2000000000LL) hold_all("arm_disable_not_confirmed");
        }
      }
    }
  }
  bool command(const std::string& s) {
    if (manual_pose_blocked_) {
      if (manual_pose_phase_ == "moving" && (s == "stop" || s == "park" || s == "disable" || s == "halt" || s == "hold")) {
        life("joint_stop:" + manual_pose_generation_);
        reason_ = "joint_motion_stop_requested"; return true;
      }
      reason_ = "manual_pose_in_progress";
      return false;
    }
    if (s == "start" && !start_manual_generation_.empty()) {
      reason_ = "waiting_for_manual_pose_handover"; return false;
    }
    if (shared_) return shared_command(s);
    if (s == "disable" || s == "halt" || s == "hold") { hold_all("operator_stop_all"); return true; }
    if (s == "stop" || s == "park" || s == "disable" || s == "halt") clear_manual_start();
    if (s == "start") {
      if ((phase_ != "disabled" && phase_ != "hold") || arrivals_.size() < 50 ||
          !luxi::fresh(quest_.receive_mono_ns, luxi::mono()) || quest_.left_tracking == 2 ||
          quest_.right_tracking == 2 || !luxi::valid_state(state_) ||
          !luxi::fresh(state_.receive_mono_ns, luxi::mono()))
        return false;
      if (manual_pose_phase_ == "holding") {
        const auto t = luxi::mono();
        if (!reserve_manual_start(t)) return false;
        return request_manual_start(t);
      }
      last_ = supported_pose(luxi::mono());
      phase_ = "homing";
      reason_ = "operator_start";
      transition_ = luxi::mono();
      life("enable");
      return true;
    }
    if (s == "stop") {
      if (phase_ == "disabled" && manual_pose_phase_ != "holding") return true;
      if (manual_pose_phase_ == "holding" && (!state_.enabled || !arm_feedback(luxi::mono()))) return false;
      if (phase_ == "returning") return true;
      return_manual_generation_ = manual_pose_phase_ == "holding" ? manual_pose_generation_ : std::string();
      return_manual_request_ns_ = 0;
      if (!begin_return(luxi::mono())) return false;
      reason_ = "operator_return";
      return true;
    }
    return false;
  }

 public:
  Manager()
      : Node("control_manager"), cfg_(luxi::config(declare_parameter<std::string>("config", ""))),
        home_cfg_(cfg_) {
    shared_ = declare_parameter("shared_enable", false);
    epoch_ = owner_ + ":0";
    home_tolerance_ = declare_parameter<double>("home_tolerance", .05);
    if (!std::isfinite(home_tolerance_) || home_tolerance_ <= 0 || home_tolerance_ > .1)
      throw std::invalid_argument("home_tolerance must be finite and in (0, 0.1] radians");
    home_joint_speed_limit_ = declare_parameter<double>("home_joint_speed_limit", 0.);
    if (!std::isfinite(home_joint_speed_limit_) || home_joint_speed_limit_ < 0.)
      throw std::invalid_argument("home_joint_speed_limit must be finite and nonnegative");
    // A separate, optional cap keeps startup slow when following is faster.
    // Short-X return uses the original timed stop sequence instead.
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
    coordinated_ = create_publisher<std_msgs::msg::String>("/luxi/coordinated_control", 1);
    base_sub_ = create_subscription<std_msgs::msg::String>("/luxi/base/status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false); if (j.is_object()) base_status_ = j;
        });
    lift_sub_ = create_subscription<std_msgs::msg::String>("/luxi/lift/status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false); if (j.is_object()) lift_status_ = j;
        });
    recorder_sub_ = create_subscription<std_msgs::msg::String>("/luxi/recorder/status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false); if (j.is_object()) menu_ = j.value("failure_menu", false);
        });
    startup_sub_ = create_subscription<std_msgs::msg::String>("/luxi/arms/startup_status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            const auto j = luxi::Json::parse(m->data);
            if (!j.is_object() || !j.contains("manual_pose") || !j.at("manual_pose").is_object()) return;
            const auto source = std::stoll(j.at("receive_mono_ns").get<std::string>());
            if (source <= manual_pose_status_ns_ || !luxi::fresh(source, luxi::mono(), 500000000)) return;
            const auto& pose = j.at("manual_pose");
            const auto next = pose.value("phase", std::string("inactive"));
            if (next != "inactive" && next != "editing" && next != "preparing" && next != "holding" && next != "moving") return;
            const auto generation = pose.value("generation", std::string());
            auto held_target = manual_pose_target_;
            if (next == "holding") {
              held_target = pose.at("target").get<std::array<double, 16>>();
              for (int i = 0; i < 16; ++i)
                if (!std::isfinite(held_target[i]) || held_target[i] < cfg_.lo[i] || held_target[i] > cfg_.hi[i]) return;
            }
            const bool entered_hold = next == "holding" &&
                (manual_pose_phase_ != "holding" || manual_pose_generation_ != generation);
            const bool was_blocked = manual_pose_blocked_;
            control_generation_ = j.value("control_generation", std::string());
            support_target_valid_ = false;
            if (j.contains("support_target")) {
              const auto target = j.at("support_target").get<std::array<double, 16>>();
              support_target_valid_ = true;
              for (int i = 0; i < 16; ++i)
                support_target_valid_ = support_target_valid_ && std::isfinite(target[i]) && target[i] >= cfg_.lo[i] && target[i] <= cfg_.hi[i];
              if (support_target_valid_) support_target_ = target;
            }
            manual_pose_status_ns_ = source;
            manual_pose_phase_ = next;
            manual_pose_generation_ = generation;
            manual_pose_target_ = held_target;
            manual_pose_fault_ = pose.value("fault_latched", false);
            manual_pose_reason_ = pose.value("reason", std::string());
            startup_owned_ = j.value("owned", false);
            manual_pose_blocked_ = next == "editing" || next == "preparing" || next == "moving";
            if (manual_pose_blocked_) suspend_for_manual_pose();
            else if (!start_manual_generation_.empty() &&
                (generation != start_manual_generation_ || startup_owned_ ||
                 (next != "holding" && !(next == "inactive" && manual_pose_reason_ == "lifecycle_handover"))))
              abort_manual_start("manual_pose_start_taken_over");
            else if (was_blocked || entered_hold) {
              // Ending an edit does not replay Quest buttons or old IK targets.
              phase_ = "disabled"; enter("stopped");
              reason_ = next == "holding" ? "manual_pose_held_waiting_for_start" : "manual_pose_exited";
              intent_ = luxi::Command{};
              return_manual_generation_.clear(); return_manual_request_ns_ = 0;
              tracking_resume_after_ = source;
              x_ = quest_.buttons[2]; long_x_ = true;
              x_released_ = x_enable_press_ = false;
            }
          } catch (const std::exception&) { /* Keep an existing edit interlock latched. */ }
        });
    states_ = create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) { state_ = *s; });
    targets_ = create_subscription<luxi::Command>(
        "/luxi/intent", 1, [this](luxi::Command::ConstSharedPtr c) { if (!manual_pose_blocked_) intent_ = *c; });
    quests_ =
        create_subscription<luxi::Quest>("/luxi/quest", 100, [this](luxi::Quest::ConstSharedPtr q) {
          if (!luxi::fresh(q->receive_mono_ns, luxi::mono())) return;
          const bool sustained_gap = !luxi::fresh(quest_.receive_mono_ns, q->receive_mono_ns,
                                                  shared_ ? input_hold_ns_ : 500000000);
          // Also catch a long outage when the recovery callback runs before
          // the watchdog timer; freshly dequeued input cannot hide the gap.
          if (shared_ && quest_.receive_mono_ns && sustained_gap &&
              system_phase_ != "disabled" && system_phase_ != "stopped" && !parking_in_progress())
            hold_all("quest_timeout");
          if (sustained_gap) {
            x_ = false;
            long_x_ = false;
            arrivals_.clear();
            x_released_ = false; x_enable_press_ = false;
          }
          if (q->receive_mono_ns <= quest_.receive_mono_ns) return;
          if (q->left_tracking != 2 && q->right_tracking != 2) {
            if (quest_.left_tracking == 2 || quest_.right_tracking == 2)
              tracking_resume_after_ = q->receive_mono_ns;
            last_tracking_ = q->receive_mono_ns;
          }
          quest_ = *q;
          arrivals_.push_back(q->receive_mono_ns);
          while (!arrivals_.empty() && q->receive_mono_ns - arrivals_.front() > 1000000000)
            arrivals_.pop_front();
          if (manual_pose_blocked_) {
            if (manual_pose_phase_ == "moving" && ((q->buttons[1] && !b_) || (q->buttons[2] && !x_))) command("halt");
            x_ = q->buttons[2]; b_ = q->buttons[1];
            x_released_ = x_enable_press_ = false; long_x_ = true;
            return;
          }
          if (shared_) {
            const bool stopped_tap = !q->buttons[2] && x_ && x_enable_press_ && !long_x_;
            if (q->buttons[1]) { if (!b_) hold_all("B_stop_all"); x_released_ = false; x_enable_press_ = false; }
            else if (!q->buttons[2]) { x_released_ = true; x_enable_press_ = false; long_x_ = false; }
            else if (!x_ && x_released_) {
              x_released_ = false; x_start_ = q->receive_mono_ns;
              x_enable_press_ = system_phase_ == "disabled" || system_phase_ == "stopped";
              long_x_ = !x_enable_press_;
              if (!x_enable_press_) command("park");
            }
            if (!q->buttons[1] && q->buttons[2] && x_enable_press_ && !long_x_ &&
                q->receive_mono_ns - x_start_ >= 1500000000) {
              long_x_ = true; command("start"); // A rejected hold is consumed, never retried automatically.
            }
            // A tap from stopped-with-torque-held requests parking, not a restart.
            if (stopped_tap && system_phase_ == "stopped" && state_.enabled)
              command("park");
            x_ = q->buttons[2]; b_ = q->buttons[1]; return;
          }
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
          res->message = shared_ ? system_phase_ + ": " + reason_ : phase_;
        });
    tick_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });
  }
  void update() {
    auto t = luxi::mono();
    double dt = previous_ ? std::min(.05, (t - previous_) * 1e-9) : .01;
    previous_ = t;
    if (manual_pose_blocked_) suspend_for_manual_pose();
    else {
      update_shared(t);
      update_manual_start(t);
    }
    input_paused_ = shared_input_paused_;
    if (phase_ != "disabled") {
      bool healthy = luxi::valid_state(state_) && luxi::fresh(state_.receive_mono_ns, t);
      if (!healthy) {
        phase_ = "fault";
        reason_ = "feedback_unavailable";
      }  // Never pretend missing motor feedback is a valid measurement.
      if (healthy && shared_input_paused_) {
        reason_ = "quest_gap";
        if (phase_ == "aligning" || phase_ == "teleop")
          tracking_resume_after_ = std::max(tracking_resume_after_, quest_.receive_mono_ns + 1);
        // Keep the same phase and submitted target during startup/teleop.
        // The operator-requested fixed return does not depend on Quest poses.
      } else if (phase_ == "returning") {
        if (healthy) advance_return(t);
      } else if (phase_ == "homing") {
        if (healthy) {
          last_ = home_cfg_.limit(cfg_.home, last_, dt);
          bool arrived = true;
          for (int i = 0; i < 16; i++)
            if (i % 8 != 7 && std::abs(state_.position[i] - cfg_.home[i]) > home_tolerance_)
              arrived = false;
          if (arrived && t - transition_ > 2000000000) {
            phase_ = "aligning";
            reason_ = "aligning";
          }
          if (t - transition_ > 30000000000) {
            phase_ = "fault";
            reason_ = "home_timeout";
          }
        }
      } else if (phase_ == "aligning" || (phase_ == "teleop" && (!shared_ || system_phase_ == "active"))) {
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
                        {"shared_enable", shared_}, {"system_phase", system_phase_},
                        {"manual_pose_active", manual_pose_blocked_},
                        {"manual_pose_phase", manual_pose_phase_},
                        {"parking_policy", "legacy_timed_return_lift_stopped"}, {"shared_epoch", epoch_},
                        {"return_nominal_duration_s", 2.0}, {"return_hz", 50},
                        {"return_step", return_step_}, {"return_steps", return_steps_},
                        {"reason", reason_},
                        {"input_paused", input_paused_},
                        {"arm_feedback_age_ms", state_.receive_mono_ns ? (t - state_.receive_mono_ns) / 1e6 : -1.0},
                        {"arm_feedback_valid", luxi::valid_state(state_)},
                        {"quest_window_samples", arrivals_.size()},
                        {"quest_age_ms", quest_.receive_mono_ns ? (t - quest_.receive_mono_ns) / 1e6 : -1.0},
                        {"intent_age_ms", intent_.source_mono_ns ? (t - intent_.source_mono_ns) / 1e6 : -1.0},
                        {"home_tolerance", home_tolerance_},
                        {"parking_requires_measured_arrival", false},
                        {"home_joint_speed_limit", home_joint_speed_limit_},
                        {"joint_speed_limits", cfg_.speed},
                        {"input_hold_timeout", input_hold_timeout_},
                        {"vr_fresh", luxi::fresh(quest_.receive_mono_ns, t)},
                        {"tracking_left", quest_.left_tracking},
                        {"tracking_right", quest_.right_tracking}}
                 .dump();
    status_->publish(s);
    broadcast();
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Manager>(); });
}
