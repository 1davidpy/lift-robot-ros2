#include <net/if.h>
// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <fcntl.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <deque>
#include <map>

#include <luxi/core.hpp>
#include "../include/luxi/lift_protocol.hpp"
#include <openarm/can/socket/openarm.hpp>
#include <std_msgs/msg/string.hpp>
#include "openarm_monitor.hpp"
#include "static_position_compensation.hpp"
#include "joint_stream.hpp"
namespace oa = openarm::can::socket;
namespace dm = openarm::damiao_motor;
class Adapter : public rclcpp::Node {
  luxi::Json json_;
  int64_t status_published_ = 0;
  std::string status_key_;
  luxi::ArmConfig cfg_;
  bool monitor_only_ = false;
  bool startup_allowed_ = false, startup_owned_ = false, startup_pending_ = false;
  bool startup_recover_ = false;
  std::array<double, 16> startup_pose_{};
  std::array<double, 16> startup_reference_{};
  std::array<double, 16> startup_nominal_{};
  luxi::StaticPositionCompensation compensation_;
  std::array<double, 16> nominal_sent_{}, mock_static_load_{};
  std::string startup_rejection_;
  static constexpr double recovery_margin_ = .15;
  static constexpr double gripper_recovery_margin_ = .02;
  bool restore_hold_ = false;
  std::array<double, 16> restore_target_{};
  double startup_speed_ = .15, startup_tolerance_ = .02;
  int64_t startup_since_ = 0, startup_arrived_ = 0;
  std::string startup_phase_ = "idle", startup_reason_ = "waiting_for_web_button";
  rclcpp::Service<luxi::Service>::SharedPtr startup_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr startup_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_status_sub_;
  luxi::Json control_status_;
  int64_t control_status_time_ = 0;
  std::string manual_phase_ = "inactive", manual_reason_ = "not_requested";
  std::string manual_generation_;
  std::array<double, 16> manual_target_{}, manual_reference_{};
  int64_t manual_since_ = 0, manual_stable_since_ = 0, command_after_ = 0;
  bool manual_disabled_ = false, manual_fault_latched_ = false;
  static constexpr double manual_velocity_limit_ = .08, manual_stability_rad_ = .01;
  static constexpr int64_t manual_settle_ns_ = 300000000, manual_timeout_ns_ = 3000000000LL;
  wheel::JointStreamGuard stream_guard_;
  wheel::JointStreamSocket stream_socket_;
  int stream_port_ = 18034;
#include "act_adapter.inc"

  bool stream_console(int64_t now) const {
    return luxi::fresh(control_status_time_, now, 500000000) &&
        control_status_.value("control_source", "") == "console" &&
        (control_status_.value("system_phase", "disabled") == "disabled" ||
         control_status_.value("system_phase", "disabled") == "stopped");
  }

  bool stream_command(const std::string& detail, std::string& message) {
    try {
      const auto packet = luxi::Json::parse(detail);
      const auto id = packet.at("request_id").get<std::string>();
      const auto key = packet.at("stream_key").get<std::string>();
      const double speed = packet.value("speed_rad_s", .2);
      const double acceleration = packet.value("acceleration_rad_s2", .5);
      const auto now = luxi::mono();
      if (id.empty() || id.size() > 96 || !enabled_ || manual_fault_latched_ ||
          joint_phase_ == "fault" || !stream_console(now))
        throw std::invalid_argument("Owned enabled console hold required; no implicit reset or enable");
      wheel::Axes speeds = cfg_.speed;
      for (auto& value : speeds) value = std::min(value, speed);
      wheel::JointStreamGuard candidate;
      candidate.begin(key, cfg_.lo, cfg_.hi, speeds, nominal_sent_, now, acceleration);
      if (!joint_command("joint_move", luxi::Json{{"request_id", id}, {"unit", "rad"},
          {"positions", nominal_sent_}, {"speed_rad_s", speed}}.dump(), message)) return false;
      if (joint_id_ != id || joint_phase_ != "preparing")
        throw std::invalid_argument("Stream identity already consumed");
      stream_guard_ = std::move(candidate);
      joint_phase_ = "streaming"; joint_reason_ = "waiting_for_laptop_joint_reference";
      message = "Laptop stream active; joint reference, feedback and image watchdogs enabled";
      return true;
    } catch (const std::exception& error) { message = error.what(); return false; }
  }

  void update_stream(int64_t now, double dt) {
    if (!manual_feedback_fresh(now) || !joint_positions_in_range(state_.position)) {
      finish_joint("fault", "joint_stream_feedback_invalid", true); return;
    }
    if (!stream_console(now)) { finish_joint("stopped", "joint_stream_authority_lost"); return; }
    stream_socket_.drain(stream_guard_, now, nominal_sent_);
    if (const auto* reason = stream_guard_.expired(now)) {
      finish_joint("stopped", reason); return;
    }
    for (int axis = 0; axis < 16; ++axis)
      if (std::abs(sent_[axis] - state_.position[axis]) > .25) {
        finish_joint("fault", "joint_stream_following_error"); return;
      }
    try {
      nominal_sent_ = stream_guard_.sample(nominal_sent_, dt);
    } catch (const std::exception&) {
      finish_joint("stopped", "joint_stream_board_reference_rejected"); return;
    }
    joint_target_ = stream_guard_.goal;
    command_.position = nominal_sent_; command_.phase = "joint_stream";
    command_.source_mono_ns = now;
    joint_reason_ = stream_guard_.sequence ? "following_laptop_joint_references" : "waiting_for_laptop_joint_reference";
  }

  std::string joint_phase_ = "idle", joint_reason_ = "not_requested", joint_id_;
  std::array<double, 16> joint_target_{}, joint_from_{}, joint_reference_{};
  double joint_speed_ = .15, joint_duration_ = 0, joint_progress_ = 0;
  int64_t joint_since_ = 0, joint_stable_since_ = 0, joint_arrived_ = 0;
  std::map<std::string, std::string> joint_requests_;
  std::deque<std::string> joint_request_order_;
  std::string control_generation_ = std::to_string(luxi::mono());
  uint64_t output_enable_count_ = 0, output_disable_count_ = 0;
  void next_control_generation() { control_generation_ = std::to_string(luxi::mono()); }
  void enable_outputs() {
    if (enabled_) return; // Ownership changes never re-enable an already powered arm.
    enabled_ = true; // Initial partial enable still participates in error cleanup.
    ++output_enable_count_;
    for (auto& a : arms_) if (a) a->enable_all();
  }
  bool joint_active() const { return joint_phase_ == "preparing" || joint_phase_ == "moving" || joint_phase_ == "streaming" || joint_phase_ == "act"; }
  bool manual_editing() const { return manual_phase_ == "editing" || manual_phase_ == "preparing" || manual_phase_ == "moving"; }
  void clear_command() {
    command_ = luxi::Command{};
    command_after_ = luxi::mono();
  }
  void leave_manual(const std::string& reason) {
    if(act_owned()) {act_stage_="stopped";act_reason_=reason;act_steps_.clear();}
    act_raw_=false;
    if (joint_phase_ != "idle") { joint_phase_ = "idle"; joint_reason_ = reason; }
    manual_phase_ = "inactive"; manual_reason_ = reason;
    manual_disabled_ = manual_fault_latched_ = false; manual_stable_since_ = 0;
  }
  void latch_manual_fault(const std::string& reason) {
    if (manual_fault_latched_) return;
    if (joint_phase_ != "idle") { joint_phase_ = "fault"; joint_reason_ = reason; }
    manual_fault_latched_ = true; manual_reason_ = reason;
    manual_generation_ = std::to_string(luxi::mono());
    next_control_generation();
    clear_command();
  }
  void writable_buses() {
    if (!monitor_only_) return;
    if (!mock_) {
      for (auto& m : monitors_) m.reset();
      for (int side = 0; side < 2; ++side) initialize_arm(side);
    }
    monitor_only_ = false;
    seen_.fill(0);
  }
  void disable_outputs() {
    // Try both buses even if the first bus fails. An explicit pose-enter must
    // transmit DISABLE even when this process did not originally enable them.
    std::string failure;
    ++output_disable_count_;
    for (auto& a : arms_) if (a) {
      try { a->disable_all(); }
      catch (const std::exception& e) { if (failure.empty()) failure = e.what(); }
    }
    enabled_ = false;
    compensation_.reset();
    next_control_generation();
    if (!failure.empty()) throw std::runtime_error(failure);
  }
  bool manual_feedback_fresh(int64_t t) const {
    if (!luxi::valid_state(state_) || !luxi::fresh(state_.receive_mono_ns, t))
      return false;
    return std::all_of(seen_.begin(), seen_.end(), [t](int64_t seen) { return luxi::fresh(seen, t); }) &&
        std::all_of(state_.velocity.begin(), state_.velocity.end(), [](double v) { return std::isfinite(v); });
  }
  std::string manual_feedback_rejection(int64_t t) const {
    if (!manual_feedback_fresh(t)) return "feedback_invalid_or_stale";
    for (int i = 0; i < 16; ++i) {
      if (state_.position[i] < cfg_.lo[i] || state_.position[i] > cfg_.hi[i])
        return "joint_outside_limits";
      if (std::abs(state_.velocity[i]) > manual_velocity_limit_)
        return "arms_still_moving";
    }
    return "";
  }
  bool joint_positions_in_range(const std::array<double, 16>& positions) const {
    for (int i = 0; i < 16; ++i)
      if (!std::isfinite(positions[i]) || positions[i] < cfg_.lo[i] || positions[i] > cfg_.hi[i]) return false;
    return true;
  }
  void finish_joint(const std::string& phase, const std::string& reason, bool fault = false) {
    if(act_owned() && phase!="ready") {act_stage_=fault?"fault":"stopped";act_reason_=reason;act_steps_.clear();}
    joint_phase_ = phase; joint_reason_ = reason;
    manual_target_ = phase == "ready" ? joint_target_ : nominal_sent_; manual_reason_ = reason;
    if (!enabled_) {
      manual_phase_ = "inactive"; clear_command(); return;
    }
    manual_phase_ = "holding";
    if (fault) latch_manual_fault(reason);
  }
  bool joint_command(const std::string& command, const std::string& detail, std::string& message) {
    if (command == "joint_stop") {
      if (!detail.empty()) {
        try {
          if (luxi::Json::parse(detail).at("request_id").get<std::string>() != joint_id_) {
            message = "Unowned joint stop rejected"; return false;
          }
        } catch (const std::exception& error) { message = error.what(); return false; }
      }
      if (joint_active()) finish_joint("stopped", "joint_operator_stop");
      message = "关节动作已停止，保留已发送姿态；故障状态需重新显式操作"; return true;
    }
    std::array<double, 16> target{};
    double speed = .15; std::string id, fingerprint;
    // Reject the entire request before changing ownership, targets or enables.
    // In particular, malformed input must never reach the hardware-error unwind.
    try {
      const auto j = luxi::Json::parse(detail);
      if (!j.is_object() || !j.contains("positions") || !j.at("positions").is_array() || j.at("positions").size() != 16 ||
          !j.contains("request_id") || !j.at("request_id").is_string()) throw std::invalid_argument("需要 request_id 和 16 个关节角度");
      id = j.at("request_id").get<std::string>();
      if (id.empty() || id.size() > 96) throw std::invalid_argument("request_id 长度须为 1–96");
      const auto unit = j.value("unit", std::string("rad"));
      if (unit != "rad" && unit != "deg") throw std::invalid_argument("接口角度单位须为 rad 或 deg");
      constexpr double degrees_per_radian = 180.0 / 3.14159265358979323846;
      for (int i = 0; i < 16; ++i) {
        if (!j.at("positions").at(i).is_number()) throw std::invalid_argument("角度必须为有限数字");
        const auto value = j.at("positions").at(i).get<double>();
        if (!std::isfinite(value)) throw std::invalid_argument("角度必须为有限数字");
        target[i] = value;
        if (unit == "deg") {
          // Compare in the supplied unit too, so conversion cannot round an
          // out-of-range degree input back onto an allowed radian endpoint.
          if (value < cfg_.lo[i] * degrees_per_radian || value > cfg_.hi[i] * degrees_per_radian)
            throw std::invalid_argument(std::string(i < 8 ? "右臂" : "左臂") + " J" + std::to_string(i % 8 + 1) + " 目标角度越界（deg）");
          target[i] = value == cfg_.lo[i] * degrees_per_radian ? cfg_.lo[i] :
                      value == cfg_.hi[i] * degrees_per_radian ? cfg_.hi[i] : value / degrees_per_radian;
        }
        if (!std::isfinite(target[i]) || target[i] < cfg_.lo[i] || target[i] > cfg_.hi[i])
          throw std::invalid_argument(std::string(i < 8 ? "右臂" : "左臂") + " J" + std::to_string(i % 8 + 1) +
              " 目标越界，限位 [" + std::to_string(cfg_.lo[i]) + ", " + std::to_string(cfg_.hi[i]) + "] rad");
      }
      if (j.contains("speed_rad_s")) {
        if (!j.at("speed_rad_s").is_number()) throw std::invalid_argument("速度必须为有限数字");
        speed = j.at("speed_rad_s").get<double>();
      }
      if (!std::isfinite(speed) || speed < .02 || speed > .2) throw std::invalid_argument("速度须在 0.02–0.2 rad/s 内");
      fingerprint = luxi::Json{{"positions", target}, {"speed_rad_s", speed}}.dump();
    } catch (const std::exception& e) { message = e.what(); return false; }
    const auto prior = joint_requests_.find(id);
    if (prior != joint_requests_.end()) {
      message = prior->second == fingerprint ? "该请求已处理，不重复执行" : "同一 request_id 不能指定不同目标";
      return prior->second == fingerprint;
    }
    const auto t = luxi::mono();
    if (joint_active() || manual_editing() || startup_pending_ || (startup_owned_ && startup_phase_ == "moving")) {
      message = "已有动作执行中，请先停止并保持"; return false;
    }
    if (control_status_.is_object() && luxi::fresh(control_status_time_, t, 500000000)) {
      const auto phase = control_status_.value("phase", "disabled");
      const auto system = control_status_.value("system_phase", "disabled");
      if (control_status_.value("shared_enable", false) ? system != "disabled" && system != "stopped" : phase != "disabled" && phase != "hold") {
        message = "请先停止遥操作，再执行关节目标"; return false;
      }
    }
    if (!manual_feedback_rejection(t).empty()) { message = "需要全部关节新鲜有效、在原限位内且停稳的反馈"; return false; }
    const auto from = enabled_ && !manual_fault_latched_ ? nominal_sent_ : state_.position;
    if (!joint_positions_in_range(from)) { message = "当前保持目标不在原限位内，不能开始关节动作"; return false; }
    writable_buses();
    act_raw_=false;
    startup_owned_ = startup_pending_ = restore_hold_ = false;
    startup_phase_ = "idle"; startup_reason_ = "joint_target_control";
    next_control_generation();
    const bool reanchor = !enabled_ || manual_fault_latched_;
    manual_phase_ = "moving"; manual_reason_ = "joint_target_preparing";
    manual_fault_latched_ = false; manual_disabled_ = false;
    manual_generation_ = std::to_string(t);
    joint_id_ = id; joint_target_ = target; joint_from_ = nominal_sent_ = from;
    if (reanchor) { sent_ = from; compensation_.reset(); }
    joint_reference_ = state_.position; joint_speed_ = speed;
    joint_progress_ = 0; joint_stable_since_ = joint_arrived_ = 0; joint_since_ = t;
    joint_duration_ = 0;
    for (int i = 0; i < 16; ++i)
      joint_duration_ = std::max(joint_duration_, std::abs(target[i] - from[i]) / std::min(speed, cfg_.speed[i]));
    joint_phase_ = "preparing"; joint_reason_ = "waiting_for_stable_feedback";
    clear_command(); command_.position = nominal_sent_; command_.phase = "joint_target_preparing"; command_.source_mono_ns = t;
    joint_requests_[id] = fingerprint; joint_request_order_.push_back(id);
    if (joint_request_order_.size() > 256) { joint_requests_.erase(joint_request_order_.front()); joint_request_order_.pop_front(); }
    message = "关节目标已接受，确认反馈稳定后缓慢执行，到位后保持"; return true;
  }
  void update_joint(int64_t t, double dt) {
    if (!joint_active()) return;
    if (joint_phase_ == "act") { update_act(t,dt); return; }
    if (joint_phase_ == "streaming") { update_stream(t, dt); return; }
    if (!manual_feedback_fresh(t)) {
      if (joint_phase_ == "moving" || enabled_ || joint_stable_since_ || t - joint_since_ > manual_timeout_ns_)
        finish_joint("fault", "joint_feedback_lost", true);
      return;
    }
    if (!joint_positions_in_range(state_.position)) { finish_joint("fault", "joint_feedback_outside_limits", true); return; }
    if (joint_phase_ == "preparing") {
      if (t - joint_since_ > manual_timeout_ns_) { finish_joint("fault", "joint_prepare_timeout"); return; }
      for (auto seen : seen_) if (seen <= joint_since_) return;
      bool stable = true;
      for (int i = 0; i < 16; ++i)
        stable = stable && std::abs(state_.position[i] - joint_reference_[i]) <= manual_stability_rad_ &&
            std::abs(state_.velocity[i]) <= manual_velocity_limit_;
      if (!stable) { joint_reference_ = state_.position; joint_stable_since_ = 0; }
      else if (!joint_stable_since_) joint_stable_since_ = t;
      if (joint_stable_since_ && t - joint_stable_since_ >= manual_settle_ns_) {
        // The start anchor remains the preloaded support target, never a stale
        // remote command or a newly clamped out-of-range measurement.
        for (int i = 0; i < 16; ++i)
          if (std::abs(state_.position[i] - joint_from_[i]) > .15) { finish_joint("fault", "joint_start_position_changed"); return; }
        if (!enabled_) {
          joint_from_ = nominal_sent_ = sent_ = state_.position;
          compensation_.reset();
          joint_duration_ = 0;
          for (int i = 0; i < 16; ++i)
            joint_duration_ = std::max(joint_duration_, std::abs(joint_target_[i] - joint_from_[i]) / std::min(joint_speed_, cfg_.speed[i]));
        }
        send_target(sent_);
        if (!enabled_) {
          enable_outputs();
        }
        send_target(sent_);
        joint_phase_ = "moving"; joint_reason_ = "joint_target_moving"; joint_since_ = t;
      }
    } else {
      // ACT homing also waits for static-load compensation to settle after
      // a raw-target episode. Keep a bounded 30 s minimum, without relaxing
      // measured arrival, velocity, limits, or following-error checks.
      const double timeout = act_stage_ == "homing" ? std::max(30., joint_duration_ * 2 + 10) : joint_duration_ * 2 + 10;
      if ((t - joint_since_) * 1e-9 > timeout) { finish_joint("fault", "joint_motion_timeout"); return; }
      for (int i = 0; i < 16; ++i)
        if (std::abs(sent_[i] - state_.position[i]) > .25) { finish_joint("fault", "joint_following_error"); return; }
      joint_progress_ = joint_duration_ < 1e-9 ? 1. : std::min(1., joint_progress_ + dt / joint_duration_);
      bool arrived = joint_progress_ >= 1.;
      for (int i = 0; i < 16; ++i) {
        nominal_sent_[i] = joint_progress_ >= 1. ? joint_target_[i] :
            std::clamp(joint_from_[i] + (joint_target_[i] - joint_from_[i]) * joint_progress_, cfg_.lo[i], cfg_.hi[i]);
        arrived = arrived && std::abs(state_.position[i] - joint_target_[i]) <= .02 && std::abs(state_.velocity[i]) < .08;
      }
      if (arrived) { if (!joint_arrived_) joint_arrived_ = t; }
      else joint_arrived_ = 0;
      if (joint_arrived_ && t - joint_arrived_ >= 500000000) finish_joint("ready", "joint_target_arrived");
    }
    if (joint_active()) {
      command_.position = nominal_sent_; command_.phase = "joint_target_" + joint_phase_;
      command_.source_mono_ns = t;
    }
  }
  bool manual_command(const std::string& command, std::string& message) {
    const auto t = luxi::mono();
    if (manual_phase_ == "moving") { message = "请先停止关节动作并保持"; return false; }
    if (command == "pose_cancel" && !manual_editing()) {
      message = "仅可取消正在编辑或准备保持的动作设置";
      return false;
    }
    if (command == "pose_enter" || command == "pose_cancel") {
      if (joint_phase_ != "idle") { joint_phase_ = "idle"; joint_reason_ = "manual_pose_takeover"; }
      startup_owned_ = startup_pending_ = restore_hold_ = false;
      startup_phase_ = "idle"; startup_reason_ = "manual_pose";
      compensation_.reset(); clear_command();
      manual_phase_ = "editing"; manual_reason_ = "disabling";
      manual_fault_latched_ = false;
      if (command == "pose_enter") manual_generation_ = std::to_string(t);
      manual_disabled_ = false; manual_stable_since_ = 0;
      writable_buses();
      disable_outputs();
      nominal_sent_ = sent_ = state_.position;
      manual_disabled_ = true;
      if (command == "pose_cancel") {
        leave_manual("operator_cancel");
        message = "已退出动作设置，双臂保持失能";
      } else {
        manual_reason_ = "motors_disabled";
        message = "双臂及夹爪已失能，请托住机械臂后手动摆姿";
      }
      return true;
    }
    if (manual_phase_ != "editing" || !manual_disabled_) {
      message = "请先进入动作设置模式并确认双臂已失能"; return false;
    }
    const auto rejection = manual_feedback_rejection(t);
    if (!rejection.empty()) {
      manual_reason_ = rejection;
      message = rejection == "joint_outside_limits" ? "当前姿态超出关节限位，保持前请手动移回限位内" :
          rejection == "arms_still_moving" ? "机械臂仍在移动，请稳定托住后再保持" : "需要全部 16 轴的有效实时反馈";
      return false;
    }
    manual_phase_ = "preparing"; manual_reason_ = "waiting_for_stable_feedback";
    manual_since_ = t; manual_stable_since_ = 0; manual_reference_ = state_.position;
    clear_command();
    message = "正在确认姿态稳定，将原地使能保持当前姿态";
    return true;
  }
  void update_manual(int64_t t) {
    if (manual_phase_ == "editing") {
      if (manual_disabled_) {
        const auto rejection = manual_feedback_rejection(t);
        manual_reason_ = rejection.empty() ? "ready_to_hold" : rejection;
      }
      return;
    }
    if (manual_phase_ == "preparing") {
      const auto rejection = manual_feedback_rejection(t);
      if (t - manual_since_ > manual_timeout_ns_) {
        manual_phase_ = "editing"; manual_reason_ = "stable_feedback_timeout";
        return;
      }
      if (!rejection.empty()) {
        manual_stable_since_ = 0; manual_reason_ = rejection;
        if (rejection != "feedback_invalid_or_stale") manual_phase_ = "editing";
        return;
      }
      // All 16 samples must have arrived after the hold button. A cached
      // pre-disable/pre-click position must never become an enable target.
      for (const auto seen : seen_) if (seen <= manual_since_) return;
      bool moved = false;
      for (int i = 0; i < 16; ++i)
        moved = moved || std::abs(state_.position[i] - manual_reference_[i]) > manual_stability_rad_;
      if (!manual_stable_since_ || moved) {
        manual_reference_ = state_.position; manual_stable_since_ = t;
      }
      manual_reason_ = "waiting_for_stable_feedback";
      if (t - manual_stable_since_ < manual_settle_ns_) return;
      manual_target_ = state_.position;
      nominal_sent_ = sent_ = manual_target_;
      compensation_.reset();
      clear_command();
      command_.position = manual_target_; command_.phase = "manual_pose_hold";
      command_.source_mono_ns = t;
      // Preload the measured target while disabled, then resend immediately
      // after enable rather than leaving the motor's previous target in force.
      send_target(manual_target_);
      enable_outputs();
      send_target(manual_target_);
      manual_phase_ = "holding"; manual_reason_ = "current_pose_held";
      manual_disabled_ = false;
    }
    if (manual_phase_ == "holding") {
      if (joint_phase_ != "idle" && !manual_fault_latched_ && !joint_positions_in_range(state_.position))
        latch_manual_fault("joint_feedback_outside_limits");
      if (!manual_fault_latched_ && !manual_feedback_fresh(t))
        latch_manual_fault("feedback_lost_latched");
      if (manual_fault_latched_) {
        command_.source_mono_ns = 0;
        return;
      }
      command_.position = manual_target_;
      command_.phase = "manual_pose_hold";
      command_.source_mono_ns = t;
      manual_reason_ = "current_pose_held";
    }
  }
  void send_target(const std::array<double, 16>& target) {
    if (joint_phase_ != "idle" && !joint_positions_in_range(target))
      throw std::runtime_error("joint control output outside original limits");
    if (mock_) return;
    for (int s = 0; s < 2; ++s) {
      std::vector<dm::MITParam> params;
      for (int i = 0; i < 7; ++i) {
        const int k = s * 8 + i;
        params.push_back(dm::MITParam{cfg_.kp[k], cfg_.kd[k], target[k] + cfg_.offset[k], 0, 0});
      }
      arms_[s]->get_arm().mit_control_all(params);
      arms_[s]->get_gripper().set_position(target[s * 8 + 7] + cfg_.offset[s * 8 + 7], 50., 1. / 4.5);
    }
  }
  void initialize_arm(int s) {
    arms_[s] = std::make_unique<oa::OpenArm>(bus_names_[s], json_.at("fd"));
    arms_[s]->init_arm_motors(
        {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340,
         dm::MotorType::DM4340, dm::MotorType::DM4310, dm::MotorType::DM4310,
         dm::MotorType::DM4310}, {1, 2, 3, 4, 5, 6, 7}, {17, 18, 19, 20, 21, 22, 23});
    arms_[s]->init_gripper_motor(dm::MotorType::DM4310, 8, 24, dm::ControlMode::POS_FORCE);
    arms_[s]->set_callback_mode_all(dm::CallbackMode::STATE);
  }
  void startup_hold(const std::string& reason) {
    startup_pending_ = false;
    startup_nominal_ = nominal_sent_;
    startup_phase_ = "hold";
    startup_reason_ = reason;
    startup_arrived_ = 0;
    // Retain the submitted target and its supporting torque. Replacing it
    // with measured position would unload the proportional controller.
  }
  bool startup_command(const std::string& command, std::string& message, const std::string& detail = "") {
    const auto t = luxi::mono();
    if(command.rfind("act_",0)==0) return act_command(command,detail,message);
    if(act_owned() && command!="stop" && command!="disable" && command!="joint_stop") {
      message="ACT owns arms; stop the ACT session before changing controller";return false;
    }
    const bool console_action = command == "joint_move" || command == "start" ||
        command == "restore_hold" || command == "pose_enter" || command == "pose_hold";
    if (console_action && control_status_.contains("control_source") &&
        control_status_.value("control_source", "legacy") != "legacy" &&
        (!luxi::fresh(control_status_time_, t, 500000000) ||
         control_status_.value("control_source", "") != "console")) {
      message = "请先在控制台切换为控制台权限"; return false;
    }
    if (command == "joint_stream_start") return stream_command(detail, message);
    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);
    if (command == "pose_enter" || command == "pose_hold" || command == "pose_cancel")
      return manual_command(command, message);
    if (command == "disable" && manual_phase_ != "inactive") {
      startup_owned_ = startup_pending_ = false;
      startup_phase_ = "idle"; startup_reason_ = "operator_disable";
      clear_command(); disable_outputs(); leave_manual("operator_disable");
      message = "已撤销双臂使能并移除转矩"; return true;
    }
    if (command == "stop" && manual_phase_ != "inactive") {
      if (joint_active()) return joint_command("joint_stop", "", message);
      if (manual_phase_ == "holding" && manual_fault_latched_) {
        message = "已停止目标发送，反馈故障仍锁存；请重新设置姿态或显式启动控制";
        return true;
      }
      if (manual_phase_ == "holding") {
        next_control_generation();
        startup_owned_ = true; startup_hold("operator_stop");
      } else {
        disable_outputs(); clear_command();
      }
      leave_manual("operator_stop");
      message = enabled_ ? "已停止，保留当前姿态转矩" : "已停止动作设置，双臂保持失能"; return true;
    }
    if (!startup_allowed_) { message = "启动姿态功能未配置"; return false; }
    if (command == "disable") {
      if (enabled_) disable_outputs();
      clear_command();
      enabled_ = startup_owned_ = startup_pending_ = false;
      startup_phase_ = "idle"; startup_reason_ = "operator_disable";
      message = "已撤销双臂使能并移除转矩"; return true;
    }
    if (command == "stop") {
      if (startup_owned_) startup_hold("operator_stop");
      message = "已停止抬臂，保留当前位置转矩"; return true;
    }
    const bool restore = command == "restore_hold";
    if (command != "start" && !restore) { message = "不支持的启动指令"; return false; }
    if (manual_editing()) { message = "请先保持当前姿态或退出动作设置"; return false; }
    if (startup_phase_ == "moving" || startup_pending_ ||
        (enabled_ && !startup_owned_ && manual_phase_ != "holding")) {
      message = "双臂正在运动或已由遥操作控制"; return false;
    }
    if (control_status_.is_object() && luxi::fresh(control_status_time_, t, 500000000) &&
        control_status_.value("phase", "disabled") != "disabled" &&
        control_status_.value("system_phase", "disabled") != "stopped") {
      message = "请先停止遥操作"; return false;
    }
    if (!luxi::valid_state(state_) || !luxi::fresh(state_.receive_mono_ns, t) ||
        (manual_fault_latched_ && !manual_feedback_fresh(t))) {
      message = "需要全部 16 个关节的有效实时反馈"; return false;
    }
    for (int i = 0; i < 16; ++i) {
      // A small passive overshoot may occur while the motors are disabled.
      // Recovery still moves towards the unchanged, in-range startup target.
      const double margin = restore ? 0. : i % 8 == 7 ? gripper_recovery_margin_ : recovery_margin_;
      if (state_.position[i] < cfg_.lo[i] - margin || state_.position[i] > cfg_.hi[i] + margin) {
        message = std::string(i < 8 ? "右臂" : "左臂") + " J" + std::to_string(i % 8 + 1) +
            " 当前位置 " + std::to_string(state_.position[i]) + " rad 超出可恢复范围；关节限位 [" +
            std::to_string(cfg_.lo[i]) + ", " + std::to_string(cfg_.hi[i]) + "] rad"; return false;
      }
    }
    if (restore) {
      std::vector<double> values;
      try { values = luxi::Json::parse(detail).get<std::vector<double>>(); }
      catch (const std::exception&) { message = "保持恢复目标必须是 16 轴有限数字数组"; return false; }
      if (values.size() != 16) { message = "保持恢复需要 16 轴目标"; return false; }
      for (int i = 0; i < 16; ++i) {
        if (!std::isfinite(values[i]) || values[i] < cfg_.lo[i] || values[i] > cfg_.hi[i] ||
            std::abs(values[i] - state_.position[i]) > .2) {
          message = "保持目标与当前位置不一致或超出限位"; return false;
        }
      }
      std::copy(values.begin(), values.end(), restore_target_.begin());
    }
    // Mode writes happen only after the explicit button, never on read-only boot.
    startup_reference_ = state_.position;
    writable_buses();
    const bool manual_fault_handover = manual_phase_ == "holding" && manual_fault_latched_;
    if (manual_fault_handover) {
      nominal_sent_ = sent_ = state_.position; compensation_.reset();
    }
    next_control_generation();
    leave_manual("startup_handover"); clear_command();
    if (manual_fault_handover) {
      command_.position = nominal_sent_; command_.phase = "startup_handover_hold";
      command_.source_mono_ns = command_after_;
      send_target(sent_);
    }
    startup_owned_ = true; startup_pending_ = !enabled_;
    restore_hold_ = restore;
    if (!enabled_) { nominal_sent_ = sent_ = state_.position; compensation_.reset(); }
    startup_nominal_ = nominal_sent_;
    startup_since_ = t; startup_arrived_ = 0; startup_recover_ = false;
    startup_phase_ = startup_pending_ ? "preparing" : restore ? "hold" : "moving";
    startup_reason_ = restore ? "calibration_hold_restored" : "operator_start";
    if (restore && !startup_pending_) { startup_nominal_ = nominal_sent_ = restore_target_; compensation_.reset(); }
    message = "开始缓慢抬臂，到达后保持（含图示夹爪位置）";
    return true;
  }
  void update_startup(int64_t t, double dt, bool healthy) {
    if (!startup_owned_) return;
    if (startup_pending_) {
      if (t - startup_since_ > 3000000000LL) {
        startup_hold("enable_feedback_timeout"); return;
      }
      if (!healthy) return;
      for (int i = 0; i < 16; ++i) {
        if (std::abs(state_.position[i] - startup_reference_[i]) > .05) {
          startup_hold("feedback_changed_before_enable"); return;
        }
      }
      nominal_sent_ = sent_ = startup_nominal_ = state_.position;
      compensation_.reset();
      send_target(sent_); enable_outputs(); send_target(sent_);
      startup_pending_ = false;
      startup_phase_ = restore_hold_ ? "hold" : "moving"; startup_since_ = t;
      if (restore_hold_) startup_nominal_ = nominal_sent_ = restore_target_;
    }
    if (!enabled_) return;
    if (!healthy) {
      startup_phase_ = "hold"; startup_reason_ = "feedback_lost";
      startup_recover_ = true; return;
    }
    if (startup_recover_) { startup_nominal_ = nominal_sent_ = sent_ = state_.position; compensation_.reset(); startup_recover_ = false; }
    if (startup_phase_ == "moving") {
      double error = 0, lag = 0, speed = 0;
      for (int i = 0; i < 16; ++i) {
        error = std::max(error, std::abs(startup_pose_[i] - state_.position[i]));
        lag = std::max(lag, std::abs(sent_[i] - state_.position[i]));
        speed = std::max(speed, std::abs(state_.velocity[i]));
      }
      if (lag > .35 || t - startup_since_ > 45000000000LL) {
        startup_hold(lag > .35 ? "following_error" : "startup_timeout");
      } else {
        for (int i = 0; i < 16; ++i) {
          const double step = std::min(cfg_.speed[i], startup_speed_) * dt;
          startup_nominal_[i] += std::clamp(startup_pose_[i] - startup_nominal_[i], -step, step);
        }
        if (error <= startup_tolerance_ && speed < .08) {
          if (!startup_arrived_) startup_arrived_ = t;
          if (t - startup_arrived_ >= 500000000) {
            startup_phase_ = "ready"; startup_reason_ = "measured_arrival";
          }
        } else startup_arrived_ = 0;
      }
    }
    command_.position = startup_nominal_;
    command_.phase = "startup_" + startup_phase_;
    command_.source_mono_ns = t;
  }
  std::array<std::string, 2> bus_names_;
  std::array<unsigned int, 2> bus_indices_{};
  bool mock_, enabled_ = false;
  std::array<std::unique_ptr<oa::OpenArm>, 2> arms_;
  std::array<std::unique_ptr<luxi::monitor::Bus>, 2> monitors_;
  std::array<int64_t, 2> monitor_retry_{};
  std::array<int, 2> locks_{{-1, -1}};
  std::array<int64_t, 16> seen_{};
  luxi::State state_;
  luxi::Command command_;
  std::array<double, 16> sent_{};
  int64_t previous_ = 0;
  rclcpp::Publisher<luxi::State>::SharedPtr pub_;
  rclcpp::Publisher<luxi::Command>::SharedPtr applied_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr errors_;
  rclcpp::Subscription<luxi::Command>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr life_;
  rclcpp::TimerBase::SharedPtr timer_;
  void receive(int side) {
    auto& collection = arms_[side]->get_master_can_device_collection();
    int fd = collection.get_socket_fd();
    for (int n = 0; n < 256; n++) {
      canfd_frame frame{};
      ssize_t size = recv(fd, &frame, sizeof(frame), MSG_DONTWAIT);
      if (size < 0) break;
      if ((size != CAN_MTU && size != CANFD_MTU) ||
          (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) || frame.len < 8)
        continue;
      uint32_t id = frame.can_id & CAN_SFF_MASK;
      if (id < 0x11 || id > 0x18) continue;
      if (size == CAN_MTU) {
        can_frame classic{};
        std::memcpy(&classic, &frame, CAN_MTU);
        collection.dispatch_frame_callback(classic);
      } else
        collection.dispatch_frame_callback(frame);
      if (frame.data[0] == id - 0x10 && frame.data[1] == 0 && frame.data[2] == 0x33)
        continue; // Parameter acknowledgements are not fresh motor measurements.
      if ((frame.data[0] & 15) != id - 0x10) continue;
      seen_[side * 8 + id - 0x11] = (frame.data[0] >> 4) <= 1 ? luxi::mono() : 0;
    }
    auto motors = arms_[side]->get_arm().get_motors();
    motors.push_back(*arms_[side]->get_gripper().get_motor());
    if (motors.size() != 8) throw std::runtime_error("motor count");
    for (int i = 0; i < 8; i++) {
      int k = side * 8 + i;
      state_.position[k] = motors[i].get_position() - cfg_.offset[k];
      state_.velocity[k] = motors[i].get_velocity();
      state_.effort[k] = motors[i].get_torque();
      state_.mos_temperature[k] = motors[i].get_state_tmos();
      state_.rotor_temperature[k] = motors[i].get_state_trotor();
    }
  }
  void error(const std::string& message) {
    std_msgs::msg::String m;
    m.data = message;
    errors_->publish(m);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message.c_str());
  }

 public:
  Adapter()
      : Node("openarm_adapter"),
        json_(luxi::config(declare_parameter<std::string>("config", ""))),
        cfg_(json_),
        mock_(declare_parameter("mock", true)) {
    if (!mock_ && !declare_parameter("allow_hardware", false))
      throw std::runtime_error(
          "Hardware requires allow_hardware:=true; default mock never opens CAN");
    monitor_only_ = declare_parameter("monitor_only", false);
    startup_allowed_ = json_.contains("startup_pose");
    if (startup_allowed_) {
      const auto& pose = json_.at("startup_pose");
      startup_speed_ = pose.at("speed_rad_s").get<double>();
      if (!std::isfinite(startup_speed_) || startup_speed_ <= 0 || startup_speed_ > .2)
        throw std::invalid_argument("startup speed must be in (0, 0.2]");
      for (int side = 0; side < 2; ++side) {
        const auto values = pose.at(side ? "left" : "right").get<std::vector<double>>();
        if (values.size() != 8) throw std::invalid_argument("startup pose needs 8 joints per arm");
        for (int i = 0; i < 8; ++i) {
          const int k = side * 8 + i;
          if (!std::isfinite(values[i]) || values[i] < cfg_.lo[k] || values[i] > cfg_.hi[k])
            throw std::invalid_argument("startup pose outside joint limits");
          startup_pose_[k] = values[i];
        }
      }
    }
    state_.position = cfg_.home;
    for (int i = 0; i < 16; i++)
      state_.position[i] = std::clamp(state_.position[i], cfg_.lo[i], cfg_.hi[i]);
    if (mock_) {
      const auto initial = declare_parameter<std::vector<double>>("mock_initial_positions", std::vector<double>{});
      if (!initial.empty()) {
        if (initial.size() != 16 || !std::all_of(initial.begin(), initial.end(), [](double v) { return std::isfinite(v); }))
          throw std::invalid_argument("mock_initial_positions needs 16 finite values");
        std::copy(initial.begin(), initial.end(), state_.position.begin());
      }
    }
    if (mock_) {
      const auto load = declare_parameter<std::vector<double>>("mock_static_load_rad", std::vector<double>{});
      if (!load.empty()) {
        if (load.size() != 16 || !std::all_of(load.begin(), load.end(), [](double v) { return std::isfinite(v) && std::abs(v) <= .15; }))
          throw std::invalid_argument("mock_static_load_rad needs 16 finite offsets within 0.15 rad");
        std::copy(load.begin(), load.end(), mock_static_load_.begin());
      }
    }
    nominal_sent_ = sent_ = state_.position;
    if (!mock_) {
      for (int s = 0; s < 2; s++) {
        auto iface = json_.at(s ? "left" : "right").at("interface").get<std::string>();
        if (iface.find_first_not_of(
                "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
            std::string::npos)
          throw std::runtime_error("invalid CAN interface");
        locks_[s] =
            open(("/tmp/luxi-" + iface + ".lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
        if (locks_[s] < 0 || flock(locks_[s], LOCK_EX | LOCK_NB) < 0)
          throw std::runtime_error("CAN owner already active");
        bus_names_[s] = iface;
        bus_indices_[s] = if_nametoindex(iface.c_str());
        if (monitor_only_) continue;
        initialize_arm(s);
      }
    }
    pub_ = create_publisher<luxi::State>("/luxi/arms/state", 5);
    applied_ = create_publisher<luxi::Command>("/luxi/applied", 10);
    errors_ = create_publisher<std_msgs::msg::String>("/luxi/faults", 20);
    sub_ = create_subscription<luxi::Command>(
        "/luxi/command", 1, [this](luxi::Command::ConstSharedPtr c) {
          if (!startup_owned_ && manual_phase_ == "inactive" &&
              c->source_mono_ns > command_after_ && luxi::fresh(c->source_mono_ns, luxi::mono()))
            command_ = *c;
        });
    startup_status_ = create_publisher<std_msgs::msg::String>("/luxi/arms/startup_status", 1);
    control_status_sub_ = create_subscription<std_msgs::msg::String>("/luxi/control_status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (j.is_object()) { control_status_ = j; control_status_time_ = luxi::mono(); }
        });
    startup_service_ = create_service<luxi::Service>("/luxi/arms/startup",
        [this](const std::shared_ptr<luxi::Service::Request> req,
               std::shared_ptr<luxi::Service::Response> res) {
          const bool was_enabled = enabled_;
          const bool explicit_release = req->command == "disable" || req->command == "pose_enter" || req->command == "pose_cancel";
          try { res->success = startup_command(req->command, res->message, req->detail);
            startup_rejection_ = res->success ? "" : res->message; }
          catch (const std::exception& e) {
            if (req->command.rfind("joint_", 0) == 0) { joint_phase_ = "fault"; joint_reason_ = "joint_hardware_error"; }
            if (was_enabled && !explicit_release) {
              // An unsuccessful handover must not unload the previous owner.
              if (manual_phase_ != "inactive") {
                manual_target_ = nominal_sent_; manual_phase_ = "holding";
                latch_manual_fault("handover_failed");
              } else if (startup_owned_) startup_hold("handover_failed");
              else {
                clear_command(); command_.position = nominal_sent_;
                command_.phase = "handover_failed_hold"; command_.source_mono_ns = luxi::mono();
              }
            } else if (req->command.rfind("pose_", 0) == 0 || manual_phase_ != "inactive") {
              clear_command(); manual_phase_ = "editing";
              manual_disabled_ = false; manual_reason_ = "disable_or_enable_failed";
              try { disable_outputs(); } catch (...) {}
            } else startup_hold("initialization_failed");
            res->success = false; res->message = e.what(); startup_rejection_ = res->message;
          }
        });
    life_ = create_subscription<std_msgs::msg::String>(
        "/luxi/driver_lifecycle", 10, [this](std_msgs::msg::String::ConstSharedPtr s) {
          if (monitor_only_) return;
          const bool was_enabled = enabled_;
          try {
            if (joint_active() && s->data == "joint_stop:" + manual_generation_) {
              finish_joint("stopped", "joint_operator_stop"); return;
            }
            const bool enable_request = s->data == "enable" || s->data.rfind("enable:", 0) == 0;
            // Plain legacy disables cannot identify their owner and may have
            // been queued before another control endpoint took over.
            const bool disable_request = s->data == "disable:" + control_generation_;
            const auto requested_at = luxi::mono();
            if (enable_request && control_status_.is_object() && control_status_.value("control_source", "legacy") == "console" &&
                luxi::fresh(control_status_time_, requested_at, 500000000)) return;
            if (enable_request && luxi::valid_state(state_) &&
                luxi::fresh(state_.receive_mono_ns, requested_at)) {
              if (manual_editing() || startup_pending_ || (startup_owned_ && startup_phase_ == "moving")) return;
              if (manual_phase_ == "holding" ? s->data != "enable:" + manual_generation_ : s->data != "enable") return;
              if (manual_fault_latched_ && !manual_feedback_fresh(requested_at)) return;
              const bool reanchor = !enabled_ || manual_fault_latched_;
              const auto handover_target = reanchor ? state_.position : nominal_sent_;
              next_control_generation();
              startup_owned_ = false; startup_phase_ = "idle";
              startup_reason_ = "teleop_handover";
              leave_manual("lifecycle_handover"); clear_command();
              // Preserve support torque until a newly issued target arrives.
              // Previously queued ArmCommands are older than command_after_.
              nominal_sent_ = handover_target; command_.position = handover_target;
              if (reanchor) { sent_ = handover_target; compensation_.reset(); }
              command_.phase = "lifecycle_hold"; command_.source_mono_ns = command_after_;
              send_target(sent_);
              enable_outputs();
              send_target(sent_);
            } else if (disable_request) {
              startup_owned_ = startup_pending_ = false;
              startup_phase_ = "idle"; startup_reason_ = "operator_disable";
              clear_command(); disable_outputs(); leave_manual("operator_disable");
            }
          } catch (const std::exception& e) {
            if (!was_enabled) {
              try { disable_outputs(); } catch (...) {}
            } else if (manual_phase_ != "inactive") {
              manual_target_ = nominal_sent_; manual_phase_ = "holding";
              latch_manual_fault("handover_failed");
            } else if (startup_owned_) startup_hold("handover_failed");
            else {
              clear_command(); command_.position = nominal_sent_;
              command_.phase = "handover_failed_hold"; command_.source_mono_ns = luxi::mono();
            }
            error(e.what());
          }
        });
    setup_act();
    stream_port_ = declare_parameter<int>("joint_stream_port", 18034);
    stream_socket_.open(declare_parameter<std::string>("joint_stream_bind", "127.0.0.1"), stream_port_);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });
  }
  ~Adapter() {
    for (auto& a : arms_)
      if (a && enabled_) try {
          a->disable_all();
        } catch (...) {
        }
    for (int fd : locks_)
      if (fd >= 0) close(fd);
  }
  void update() {
    auto t = luxi::mono();
    if (joint_active() && previous_ && t - previous_ > 150000000)
      finish_joint("fault", "joint_control_loop_stalled", true);
    if (manual_phase_ == "holding" && previous_ && t - previous_ > 150000000)
      latch_manual_fault("control_loop_stalled");
    double dt = previous_ ? std::min(.05, (t - previous_) * 1e-9) : .01;
    previous_ = t;
    try {
      if (mock_) {
        for (int i = 0; i < 16; i++) {
          double d = enabled_ ? std::clamp(sent_[i] - mock_static_load_[i] - state_.position[i], -cfg_.speed[i] * dt,
                                           cfg_.speed[i] * dt)
                              : 0;
          state_.position[i] += d;
          state_.velocity[i] = d / dt;
          seen_[i] = t;
        }
      } else if (monitor_only_) {
        for (int s = 0; s < 2; ++s) {
          try {
            if (!monitors_[s] && t >= monitor_retry_[s])
              monitors_[s] = std::make_unique<luxi::monitor::Bus>(bus_names_[s]);
            if (!monitors_[s]) continue;
            monitors_[s]->tick(t);
            for (int i = 0; i < 8; ++i) {
              const auto& m = monitors_[s]->motors[i];
              const int k = s * 8 + i;
              seen_[k] = m.calibrated() && m.healthy() ? m.seen : 0;
              if (!seen_[k]) continue;
              state_.position[k] = m.position() - cfg_.offset[k];
              state_.velocity[k] = m.velocity();
              state_.effort[k] = m.torque();
              state_.mos_temperature[k] = m.data[6];
              state_.rotor_temperature[k] = m.data[7];
            }
          } catch (const std::exception& e) {
            // A disconnected arm must not suppress feedback from the other bus.
            monitors_[s].reset();
            monitor_retry_[s] = t + 2000000000;
            for (int i = 0; i < 8; ++i) seen_[s * 8 + i] = 0;
            error(e.what());
          }
        }
      } else
        for (int s = 0; s < 2; s++) {
          if (if_nametoindex(bus_names_[s].c_str()) != bus_indices_[s])
            throw std::runtime_error("CAN interface disconnected or replaced; reconnect required");
          arms_[s]->refresh_all();
          receive(s);
        }
      auto feedback_now = luxi::mono();
      for (int i = 0; i < 16; i++) {
        state_.age_ns[i] = seen_[i] ? feedback_now - seen_[i] : INT64_MAX;
        state_.valid[i] = luxi::fresh(seen_[i], feedback_now) && std::isfinite(state_.position[i]);
      }
      t = feedback_now;
      act_watchdog(t);
      update_joint(t, dt);
      update_manual(t);
      update_startup(t, dt, luxi::valid_state(state_));
      // A new control client may take time to send its first target. Continue
      // the last support setpoint rather than depending on motor watchdogs.
      if (enabled_ && !startup_owned_ && manual_phase_ == "inactive" &&
          !luxi::fresh(command_.source_mono_ns, t) && manual_feedback_fresh(t) && joint_positions_in_range(sent_)) {
        command_.position = nominal_sent_; command_.phase = "controller_gap_hold"; command_.source_mono_ns = t;
      }
      state_.phase = monitor_only_ ? "monitor_only" : (enabled_ ? command_.phase : "disabled");
      if (enabled_ && !(manual_phase_ == "holding" && manual_fault_latched_) &&
          luxi::valid_state(state_) && luxi::fresh(command_.source_mono_ns, t)) {
        const auto previous_sent = sent_, previous_nominal = nominal_sent_;
        auto nominal = cfg_.limit(command_.position, previous_nominal, dt);
        auto output_speed = cfg_.speed;
        for (int i = 0; i < 16; ++i) {
          if (startup_owned_) output_speed[i] = std::min(output_speed[i], startup_speed_);
          if (joint_phase_ != "idle") output_speed[i] = std::min(output_speed[i], joint_speed_);
          // Preserve the existing gradual startup recovery from a small
          // passive overshoot; never jump an out-of-limit anchor to a bound.
          if (startup_owned_) nominal[i] = previous_nominal[i] + std::clamp(
              command_.position[i] - previous_nominal[i], -output_speed[i]*dt, output_speed[i]*dt);
        }
        nominal_sent_ = nominal;
        const bool learn = manual_feedback_fresh(t) && command_.phase != "returning" &&
            (!joint_active() || joint_phase_ == "streaming" || (joint_phase_ == "moving" && joint_progress_ >= 1.));
        auto target = act_raw_ ? nominal : compensation_.apply(nominal, state_.position, state_.velocity,
            cfg_.lo, cfg_.hi, output_speed, dt, learn);
        for (int i = 0; i < 16; ++i)
          target[i] = previous_sent[i] + std::clamp(target[i] - previous_sent[i], -output_speed[i]*dt, output_speed[i]*dt);
        send_target(target);
        sent_ = target;
        auto applied = command_;
        applied.position = sent_;
        applied.header.stamp = now();
        applied.source_mono_ns = luxi::mono();
        applied_->publish(applied);
      } else if (enabled_) {
        compensation_.freeze();
        state_.phase = manual_phase_ == "holding" && manual_fault_latched_
            ? "manual_pose_fault_latched" : "hold_feedback_or_command_timeout";
      } else compensation_.reset();
    } catch (const std::exception& e) {
      if (joint_active()) finish_joint("fault", "joint_hardware_error", true);
      state_.valid.fill(false);
      state_.phase = "fault";
      error(e.what());
      if (!mock_) throw; // Unwind, release CAN sockets; systemd retries without enabling motors.
    }
    state_.enabled = enabled_;
    state_.header.stamp = now();
    state_.header.frame_id = "arm_origin";
    state_.receive_mono_ns = luxi::mono();
    pub_->publish(state_);
    // Motor feedback stays at 100 Hz. Serialize the large diagnostic report
    // at most 20 Hz unless an operator-visible state/ownership/fault changes.
    const auto report_now = luxi::mono();
    const auto report_key = startup_phase_ + "\n" + startup_reason_ + "\n" +
        startup_rejection_ + "\n" + manual_phase_ + "\n" + manual_reason_ + "\n" +
        manual_generation_ + "\n" + joint_phase_ + "\n" + joint_reason_ + "\n" +
        joint_id_ + "\n" + control_generation_ + (enabled_ ? "\nenabled" : "\ndisabled") +
        (startup_owned_ ? "\nowned" : "\nunowned") +
        (manual_fault_latched_ ? "\nfault" : "\nok");
    if (report_key == status_key_ && report_now - status_published_ < 50000000) return;
    status_key_ = report_key;
    status_published_ = report_now;
    double max_error = 0;
    for (int i = 0; i < 16; ++i)
      max_error = std::max(max_error, std::abs(startup_pose_[i] - state_.position[i]));
    std_msgs::msg::String status;
    status.data = luxi::Json{{"configured", startup_allowed_}, {"owned", startup_owned_},
        {"control_generation", control_generation_}, {"support_target", sent_}, {"nominal_target", nominal_sent_},
        {"output_enable_count", output_enable_count_}, {"output_disable_count", output_disable_count_},
        {"last_rejection", startup_rejection_}, {"recovery_margin_rad", recovery_margin_},
        {"gripper_recovery_margin_rad", gripper_recovery_margin_},
        {"phase", startup_phase_}, {"reason", startup_reason_}, {"target", startup_pose_},
        {"speed_rad_s", startup_speed_}, {"tolerance_rad", startup_tolerance_},
        {"static_compensation_rad", compensation_.bias},
        {"act", {{"protocol",1},{"session_id",act_session_},{"stage",act_stage_},{"reason",act_reason_},
          {"raw_post_compensation",act_raw_},{"chunk_id",std::to_string(act_chunk_id_)},{"target_step",act_step_},
          {"initialization_order","arms_then_lift"},{"initial_positions",act_initial_},{"lift_target_um",-64543},{"execute_steps",60},{"fps",30},
          {"max_speed_rad_s",.2},{"heartbeat_timeout_ms",1000},{"action_timeout_ms",3000}}},
        {"static_compensation", {{"scope", "non_act_control_modes"}, {"arm_joints_enabled", true},
            {"grippers_enabled", false}, {"max_bias_rad", luxi::StaticPositionCompensation::max_bias},
            {"max_rate_rad_s", luxi::StaticPositionCompensation::max_rate},
            {"deadband_rad", luxi::StaticPositionCompensation::deadband}, {"settle_seconds", luxi::StaticPositionCompensation::settle_seconds}}},
        {"joint_control", {{"joint_stream_protocol", 1}, {"joint_stream_fixed_target_protocol", 1}, {"joint_stream_planned_trajectory_protocol", 1}, {"joint_stream_port", stream_port_},
                           {"joint_stream_image_max_age_ms", wheel::stream_image_age_ns / 1000000},
                           {"joint_stream_speed_limits", cfg_.speed},
                           {"joint_stream_sequence", stream_guard_.sequence},
                           {"phase", joint_phase_}, {"reason", joint_reason_}, {"request_id", joint_id_},
                           {"target", joint_target_}, {"speed_rad_s", joint_speed_},
                           {"lower", cfg_.lo}, {"upper", cfg_.hi}, {"unit", "rad"},
                           {"joint_order", "right_then_left"}, {"min_speed_rad_s", .02}, {"max_speed_rad_s", .2}}},
        {"manual_pose", {{"phase", manual_phase_}, {"reason", manual_reason_},
                         {"generation", manual_generation_},
                         {"fault_latched", manual_fault_latched_},
                         {"target", manual_target_},
                         {"can_hold", manual_phase_ == "editing" && manual_disabled_ &&
                                      manual_feedback_rejection(luxi::mono()).empty()}}},
        {"max_error_rad", max_error}, {"receive_mono_ns", std::to_string(luxi::mono())}}.dump();
    startup_status_->publish(status);
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Adapter>(); });
}
