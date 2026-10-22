// SPDX-License-Identifier: Apache-2.0
// STM32 lifting_platform_servo wire v1. Finite PP moves, never a velocity stream.
#include <luxi/core.hpp>
#include <luxi/external_control.hpp>
#include <luxi/coordinated_control.hpp>
#include <luxi/lift_protocol.hpp>
#include <luxi/lift_socket.hpp>
#include <luxi/lift_uart.hpp>
#include <sys/random.h>
#include <optional>

namespace lp = luxi::lift;
class Lift : public rclcpp::Node {
  struct Pending {
    can_frame frame{};
    int64_t first = 0, last = 0;
    int attempts = 1;
    bool queued = false;
  };
  int node_, margin_;
  bool dashboard_available_ = false, dashboard_position_ = false;
  bool external_mode_ = false, external_active_ = false, external_finishing_ = false;
  luxi::ExternalControl external_;
  uint64_t external_task_ = 0;
  int32_t external_target_ = 0;
  int64_t external_deadline_ = 0;
  std::string external_result_ = "idle";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr external_sub_;
  void external_command(const std::string& data) {
    if (!external_mode_) return;
    try {
      const auto t = luxi::mono();
      const auto j = luxi::Json::parse(data);
      const auto op = external_.accept(j, t);
      if (op == "stop") {
        cancel("external_stop");
      } else if (op == "arm") {
        if (enabled_) throw std::runtime_error("lift already enabled");
        external_task_ = 0; external_result_ = "idle";
        configure(false);
      } else if (op == "heartbeat") {
        if (!enabled_) throw std::runtime_error("external lift permission is off");
      } else if (op == "move") {
        // Queue one goal while idle and healthy. A background INFO may be in
        // flight, and healthy locked feedback can exceed the strict motion-age
        // mask. wait_for_settling()/next_move() still require fresh READY and
        // strict stationary feedback before any UART MOVE is transmitted.
        if (!enabled_ || phase_ != "idle" || external_active_ ||
            (pending_ && pending_->frame.data[0] != lp::Info) ||
            !link_healthy(t) || !sample_->stationary() || (sample_->flags & 64))
          throw std::runtime_error("external lift MOVE requires healthy idle lock and no session");
        if (!j.contains("task_id") || !j["task_id"].is_number_unsigned() ||
            !j.contains("target_um") || !j["target_um"].is_number_integer())
          throw std::runtime_error("invalid lift task/target");
        const auto id = j["task_id"].get<uint64_t>();
        if (j["target_um"].is_number_unsigned() && j["target_um"].get<uint64_t>() > 180000)
          throw std::runtime_error("external target integer overflow/range");
        const auto target = j["target_um"].get<int64_t>();
        const int64_t lower = int64_t(std::max(configuration_[2], soft_min_)) + (dashboard_position_ ? 0 : margin_);
        const int64_t upper = int64_t(std::min(configuration_[3], soft_max_)) - (dashboard_position_ ? 0 : margin_);
        if (!id || id <= external_task_ || target < lower || target > upper)
          throw std::runtime_error("external lift target/id out of range; no clipping/replay");
        external_task_ = id; external_target_ = static_cast<int32_t>(target);
        external_active_ = true; external_finishing_ = false; external_result_ = "running";
        external_deadline_ = t + 180000000000LL;
        wait_for_settling();
      } else throw std::runtime_error("unsupported external lift operation");
    } catch (const std::exception& e) {
      // Late heartbeats after a rejection must not erase the original cause.
      if (enabled_ || !external_.revoked) cancel(e.what());
      else event("external_rejected", e.what());
    }
  }
  bool uart_mode_;
  int32_t soft_min_, soft_max_;
  bool ready_ = false, startup_released_ = false;
  int64_t ready_ns_ = 0, motion_started_ = 0, ready_after_ = 0;
  int32_t startup_origin_ = 0;
  std::string interface_, phase_ = "disabled", reason_ = "operator_enable_required";
  bool allow_motion_, enabled_ = false, session_ = false, neutral_ = false;
  bool coordinated_mode_ = false, want_home_ = false, home_complete_ = false;
  int home_locked_samples_ = 0;
  int64_t home_closed_ns_ = 0;
  luxi::CoordinatedControl coordination_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr coordination_sub_;
  bool left_ = false, right_ = false, grips_released_ = false, input_ok_ = false, menu_ = false;
  bool input_recovery_ = false;
  int64_t recovery_neutral_since_ = 0;
  int direction_ = 0, page_ = 0, ticks_ = 0, reached_count_ = 0, stopped_count_ = 0;
  int64_t quest_time_ = 0, last_heartbeat_ = 0, ack_time_ = 0, move_deadline_ = 0;
  int64_t stopped_uptime_ = -1, next_info_ = 0, session_started_ = 0;
  int64_t settled_since_ = 0, settled_last_ = 0, settle_deadline_ = 0;
  uint32_t settled_first_uptime_ = 0, settled_last_uptime_ = 0;
  uint64_t run_ = 0, action_id_ = 0;
  uint64_t can_error_frames_ = 0, can_frame_diagnostics_ = 0;
  const int64_t process_started_ = luxi::mono();
  uint8_t next_sequence_ = 0;
  std::array<int64_t, 256> used_{};
  std::array<int32_t, 10> configuration_{};
  int32_t nonce_ = 0, target_ = 0;
  std::optional<Pending> pending_, heartbeat_;
  std::optional<lp::Sample> sample_, previous_reached_;
  lp::Assembler assembler_;
  std::unique_ptr<lp::Socket> socket_;
  std::unique_ptr<lp::Uart> uart_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, raw_pub_, events_pub_, status_pub_;
  rclcpp::Subscription<luxi::Quest>::SharedPtr quest_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recorder_sub_;
  rclcpp::Service<luxi::Service>::SharedPtr control_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, luxi::Json j) {
    j["node"] = node_; j["run_segment"] = run_; j["mock"] = false;
    j["process_started_mono_ns"] = std::to_string(process_started_);
    j["transport"] = uart_mode_ ? "uart" : "can";
    std_msgs::msg::String m; m.data = j.dump(); pub->publish(m);
  }
  void event(const std::string& type, const std::string& detail) {
    publish(events_pub_, {{"schema", "lift_event_v1"}, {"event", type}, {"detail", detail},
                         {"action_id", action_id_}, {"receive_mono_ns", std::to_string(luxi::mono())}});
  }
  uint8_t sequence() {
    auto t = luxi::mono();
    for (int i = 0; i < 256; ++i) {
      auto seq = next_sequence_++;
      if (!used_[seq] || t - used_[seq] >= 1000000000) { used_[seq] = t; return seq; }
    }
    throw std::runtime_error("lift sequence exhausted");
  }
  void transmit(const can_frame& f, const char* purpose) {
    if (uart_mode_) uart_->send(f); else socket_->send(f);
    publish(events_pub_, {{"schema", "lift_command_v1"}, {"event", "tx"}, {"purpose", purpose},
                         {"action_id", action_id_}, {"opcode", f.data[0]}, {"sequence", f.data[1]},
                         {"value", lp::i32(f.data + 2)}, {"bytes", std::vector<uint8_t>(f.data, f.data + 8)},
                         {"tx_mono_ns", std::to_string(luxi::mono())},
                         {"source_quest_mono_ns", std::to_string(quest_time_)}});
  }
  void request(uint8_t opcode, int32_t value = 0) {
    if (pending_) throw std::runtime_error("lift command already pending");
    auto f = lp::request(node_, opcode, sequence(), value, uart_mode_);
    auto t = luxi::mono();
    pending_ = Pending{f, t, t, 1, false};
    if (opcode == lp::Move || opcode == lp::Home) {
      motion_started_ = t; startup_origin_ = sample_->current_um; startup_released_ = false;
      ready_ = false; ready_ns_ = 0;
    }
    transmit(f, "request");
  }
  bool feedback(int64_t t) const { return sample_ && sample_->valid(t); }
  bool startup_grace(int64_t t) const {
    return uart_mode_ && !startup_released_ &&
           (phase_ == "move_ack" || phase_ == "moving" || (enabled_ && phase_ == "stopping")) &&
           sample_ && sample_->startup_healthy(t, motion_started_, startup_origin_);
  }
  bool link_healthy(int64_t t) const { return sample_ && (sample_->healthy(t) || startup_grace(t)); }
  bool command_ready(int64_t t) const {
    return !uart_mode_ || (ready_ && ready_ns_ >= ready_after_ && t >= ready_ns_ && t - ready_ns_ <= 500000000);
  }
  std::string health_reason(int64_t t) const {
    if (!sample_ || t < sample_->received_ns || t - sample_->first_ns > 300000000)
      return uart_mode_ ? "uart_feedback_timeout" : "can_feedback_timeout";
    if (sample_->fault) return "drive_fault";
    if (sample_->flags & 128) return "motion_inhibited";
    if (!(sample_->flags & 1)) return "position_invalid";
    if (!uart_mode_ && (sample_->guard & 4)) return "can_bus_off";
    if (sample_->age_ms(t) > 500) return startup_grace(t) ? "locked_startup_preparation" : "motor_feedback_stale";
    return "ok";
  }
  bool input_present(int64_t t) const { return !menu_ && (external_mode_ ? external_.live(t) : (input_ok_ && luxi::fresh(quest_time_, t, 200000000))); }
  bool input(int64_t t) const { return !input_recovery_ && input_present(t); }
  bool stationary(int64_t t) const { return feedback(t) && sample_->stationary(); }
  void reset_settling() { settled_since_ = settled_last_ = 0; }
  void observe_settling(const lp::Sample& s, int64_t t) {
    // The v21 firmware requires 1200 ms after StopMotion(), beyond its
    // brake-locked/REACHED indication. Observe a full additional dwell rather
    // than treating two REACHED packets as permission for another command.
    if (!s.healthy(t) || !s.stationary()) { reset_settling(); return; }
    const uint32_t elapsed = s.uptime_ms - settled_last_uptime_;
    if (!settled_since_ || s.first_ns - settled_last_ > 300000000 ||
        elapsed == 0 || elapsed > 300) {
      settled_since_ = s.first_ns; settled_first_uptime_ = s.uptime_ms;
    }
    settled_last_ = s.first_ns; settled_last_uptime_ = s.uptime_ms;
  }
  bool motion_settled(int64_t t) const {
    if (uart_mode_) return stationary(t) && command_ready(t);
    return stationary(t) && settled_since_ && t - settled_last_ <= 300000000 &&
           settled_last_ - settled_since_ >= 1200000000 &&
           uint32_t(settled_last_uptime_ - settled_first_uptime_) >= 1200;
  }
  void wait_for_settling() {
    phase_ = "waiting_for_settle"; reason_ = "waiting_for_firmware_settle";
    settle_deadline_ = luxi::mono() + 5000000000LL;
    if (uart_mode_) { ready_ = false; ready_after_ = luxi::mono(); next_info_ = 0; }
  }
  bool home_locked(int64_t t) const {
    return home_complete_ && home_locked_samples_ >= 2 && link_healthy(t) && sample_->stationary() &&
           !(sample_->flags & 64) && std::abs(int64_t(sample_->current_um)) <= 10;
  }
  void configure(bool home) {
    const auto t = luxi::mono();
    if (!allow_motion_ || !input(t) || !link_healthy(t) || !sample_->stationary() || (sample_->flags & 64) ||
        (!external_mode_ && !home && !grips_released_)) throw std::runtime_error("lift prepare requires motion permission, fresh stopped feedback and neutral input");
    enabled_ = true; neutral_ = false; want_home_ = home;
    home_complete_ = false; home_locked_samples_ = 0;
    page_ = 0; configuration_.fill(0); ++run_;
    ready_ = false; ready_ns_ = 0; ready_after_ = t;
    phase_ = "configuring"; reason_ = "reading_configuration_no_motion"; next_info_ = 0;
  }
  void coordinate(const std::string& data) {
    if (!coordinated_mode_) return;
    const auto t = luxi::mono();
    if (dashboard_position_) {
      const bool changed = coordination_.accept(data, t);
      if (luxi::fresh(coordination_.sent_ns, t, 250000000) && coordination_.phase != "disabled" &&
          (coordination_.phase != "halt" || changed)) {
        cancel("shared_control_requested_during_dashboard_position"); coordination_.blocked = true;
      }
      return;
    }
    const auto old_epoch = coordination_.epoch;
    const bool changed = coordination_.accept(data, t);
    if (!coordination_.live(t)) {
      if (enabled_ || nonce_) cancel("coordinator_lost_or_stopped");
      return;
    }
    if (!changed) return;
    try {
      if (old_epoch != coordination_.epoch) { home_complete_ = false; home_locked_samples_ = 0; }
      if (coordination_.phase == "prepare") { cancel("shared_prepare"); configure(false); }
      else if (coordination_.phase == "active") {
        if (!enabled_ || phase_ != "idle" || !link_healthy(t) || !sample_->stationary() || !input_present(t) || !grips_released_)
          throw std::runtime_error("lift shared enable rejected");
        neutral_ = false; reason_ = "release_both_grips_then_press";
      } else if (coordination_.phase == "home") { cancel("shared_home_prepare"); configure(true); }
      else { cancel("shared_" + coordination_.phase); }
    } catch (const std::exception& e) { coordination_.blocked = true; cancel(e.what()); }
  }
  void cancel(const std::string& why, bool keep_permission = false) {
    const auto lease_age = external_.received_ns ? (luxi::mono() - external_.received_ns) / 1000000 : -1;
    RCLCPP_WARN(get_logger(), "lift cancel: %s; phase=%s; position_um=%d; target_um=%d; external_age_ms=%lld; lease_ms=%lld",
                why.c_str(), phase_.c_str(), sample_ ? sample_->current_um : 0, target_,
                static_cast<long long>(lease_age), static_cast<long long>(external_.lease_ns / 1000000));
    const bool owned = nonce_ != 0; // Includes uncertain HELLO/MOVE outcomes.
    pending_.reset(); heartbeat_.reset(); session_ = false;
    enabled_ = keep_permission && enabled_;
    if (external_mode_ && !enabled_) {
      external_.revoke(); external_active_ = external_finishing_ = false;
      if (external_task_) external_result_ = "stopped";
    }
    if (dashboard_position_ && !enabled_) { dashboard_position_ = false; external_mode_ = false; }
    input_recovery_ = false; recovery_neutral_since_ = 0;
    want_home_ = false;
    direction_ = 0; neutral_ = false; previous_reached_.reset(); reached_count_ = 0;
    stopped_count_ = 0; stopped_uptime_ = -1;
    reset_settling();
    // A normal release still closes the session immediately. While waiting
    // for its fresh stopped sample, retain only the ORIGINAL bounded locked
    // startup window; never restart/extend it or allow a new move from it.
    if (!enabled_) motion_started_ = 0;
    ready_ = false; ready_ns_ = 0;
    reason_ = why; phase_ = enabled_ ? "stopping" : "disabled";
    auto nonce = nonce_; nonce_ = 0;
    // CLOSE stops only this transport's owned motion. Do not override independent local
    // buttons/boot homing with an implicit global STOP.
    if (owned) {
      try { transmit(lp::request(node_, lp::Close, sequence(), nonce, uart_mode_), "cancel_remote"); }
      catch (const std::exception& e) { reason_ += "; CLOSE failed, lease expiry pending: " + std::string(e.what()); }
    }
    event("cancel", reason_);
  }
  void pause_input() {
    if (!enabled_) return;
    if (want_home_) {
      // Legacy explicit HOME is a discrete operation, not a held-grip task.
      // Cancel it without automatic replay; normal X parking never uses HOME.
      if (coordinated_mode_) coordination_.blocked = true;
      cancel("quest_gap_during_home");
      return;
    }
    recovery_neutral_since_ = 0;
    if (input_recovery_) return;
    // Read-only discovery and an idle, session-free lift stay ready while
    // arms home/align. Only an actual/pending grip operation needs canceling.
    if (phase_ != "configuring" && !(phase_ == "idle" && !nonce_ && !pending_))
      cancel("quest_gap_release_grips", true);
    input_recovery_ = true; neutral_ = false;
    left_ = right_ = false; grips_released_ = false;
    event("input_pause", "fresh_released_grips_required");
  }
  void start_session() {
    uint32_t bits = 0;
    if (getrandom(&bits, sizeof(bits), 0) != sizeof(bits)) throw std::runtime_error("lift nonce unavailable");
    nonce_ = static_cast<int32_t>(bits & 0x7fffffffu);
    if (!nonce_) nonce_ = 1;
    ++run_; session_started_ = luxi::mono();
    phase_ = "connecting"; reason_ = "hello_pending";
    request(lp::Hello, nonce_);
  }
  void next_move() {
    const auto t = luxi::mono();
    if (!enabled_ || !input(t) || !link_healthy(t) || !session_ || (!direction_ && !want_home_ && !external_active_))
      throw std::runtime_error("lift not ready for next finite move");
    if (!motion_settled(t)) return; // Fresh position plus firmware's stopped dwell.
    if (external_mode_ && external_active_) {
      target_ = external_target_; ++action_id_; previous_reached_.reset(); reached_count_ = 0;
      phase_ = "move_ack"; reason_ = "external_position_pending";
      move_deadline_ = external_deadline_; reset_settling();
      request(lp::Move, target_); return;
    }
    if (want_home_) {
      if (!coordinated_mode_ || !coordination_.live(t) || coordination_.phase != "home" ||
          configuration_[2] + margin_ > 0 || configuration_[3] - margin_ < 0 || !(configuration_[9] & 4))
        throw std::runtime_error("saved lift center not available inside configured limits");
      target_ = 0; ++action_id_; previous_reached_.reset(); reached_count_ = 0;
      phase_ = "move_ack"; reason_ = "saved_home_pending"; move_deadline_ = t + 180000000000LL;
      reset_settling();
      request(lp::Home); return;
    }
    if ((direction_ < 0 && ((sample_->guard & 1) || (!uart_mode_ && (sample_->raw_mask & 1)))) ||
        (direction_ > 0 && ((sample_->guard & 2) || (!uart_mode_ && (sample_->raw_mask & 8))))) {
      cancel("direction_limit", true); return;
    }
    // One finite target per held grip, as with the firmware's physical keys.
    // Releasing closes our session; heartbeats sustain only this current hold.
    // Do not stream 1 mm targets: arrival precedes firmware command readiness,
    // causing BUSY and repeated motor startup/settling pauses.
    const int64_t proposed = direction_ > 0 ? int64_t(std::min(configuration_[3], soft_max_)) - (dashboard_position_ ? 0 : margin_) :
                                            int64_t(std::max(configuration_[2], soft_min_)) + (dashboard_position_ ? 0 : margin_);
    if ((direction_ > 0 && sample_->current_um >= proposed - 10) ||
        (direction_ < 0 && sample_->current_um <= proposed + 10)) {
      cancel("interior_target_boundary_release_grips", true); return;
    }
    target_ = static_cast<int32_t>(proposed); ++action_id_;
    previous_reached_.reset(); reached_count_ = 0;
    phase_ = "move_ack"; reason_ = "held_grip_target_pending";
    move_deadline_ = t + 180000000000LL;
    reset_settling();
    request(lp::Move, target_);
  }
  void ack(const can_frame& f, int64_t received) {
    const auto op = f.data[0], seq = f.data[1], result = f.data[2], flags = f.data[3];
    const auto value = lp::i32(f.data + 4);
    publish(events_pub_, {{"schema", "lift_ack_v1"}, {"event", "ack"}, {"opcode", op},
                         {"sequence", seq}, {"result", result}, {"flags", flags}, {"value", value},
                         {"action_id", action_id_}, {"receive_mono_ns", std::to_string(received)}});
    if (op == lp::Keepalive) {
      if (heartbeat_ && heartbeat_->frame.data[1] == seq && received >= heartbeat_->first) {
        if (result || value != nonce_ || !(flags & 64)) throw std::runtime_error("lift heartbeat rejected");
        heartbeat_.reset();
      }
      return;
    }
    if (!pending_ || pending_->frame.data[0] != op || pending_->frame.data[1] != seq || received < pending_->first) return;
    // Rejections can carry zero instead of echoing the requested value.
    if (result != 0 && result != 0xfe) throw std::runtime_error("lift request rejected, result=" + std::to_string(result));
    if (op != lp::Info && value != lp::i32(pending_->frame.data + 2)) return;
    if (result == 0xfe) { pending_->queued = true; return; }
    if (result) throw std::runtime_error("lift request rejected, result=" + std::to_string(result));
    const auto requested_value = lp::i32(pending_->frame.data + 2);
    pending_.reset();
    if (op == lp::Info) {
      if (uart_mode_ && requested_value == 25) {
        ready_ = value == 1; ready_ns_ = received; next_info_ = luxi::mono() + 200000000;
        if (phase_ == "configuring" && page_ == 10 && ready_) {
          phase_ = "idle"; reason_ = "release_both_grips_then_press"; neutral_ = false;
          if (want_home_) wait_for_settling();
        }
        return;
      }
      configuration_[page_++] = value; next_info_ = luxi::mono() + 100000000;
      if (page_ == 10) {
        if (configuration_[0] != 1 || configuration_[1] < (uart_mode_ ? 29 : 20) || configuration_[4] != 2 ||
            configuration_[2] < (uart_mode_ ? -180000 : -100000) || configuration_[3] > (uart_mode_ ? 180000 : 200000) ||
            (uart_mode_ && (configuration_[2] != -180000 || configuration_[3] != 180000 || configuration_[9] != 4)) ||
            configuration_[2] >= configuration_[3] ||
            std::min(configuration_[3], soft_max_) - std::max(configuration_[2], soft_min_) <= 2 * margin_)
          throw std::runtime_error("lift protocol/firmware/reference/range unsupported");
        phase_ = uart_mode_ ? "configuring" : "idle";
        reason_ = uart_mode_ ? "waiting_for_READY" : "release_both_grips_then_press"; neutral_ = false;
        publish(events_pub_, {{"schema", "lift_configuration_v1"}, {"event", "configuration"},
                             {"info_pages_0_to_9", configuration_},
                             {"receive_mono_ns", std::to_string(luxi::mono())}});
        if (want_home_ && !uart_mode_) wait_for_settling();
      }
    } else if (op == lp::Hello) {
      if (!(flags & 64)) throw std::runtime_error("HELLO did not establish session");
      session_ = true; last_heartbeat_ = 0; phase_ = "session_ready";
      if (uart_mode_) { ready_ = false; ready_after_ = received; next_info_ = 0; }
      assembler_.clear_pending();
    } else if (op == lp::Move || op == lp::Home) {
      phase_ = "moving"; reason_ = want_home_ ? "waiting_for_locked_arrival" : "held_grip_travel";
      ack_time_ = luxi::mono(); assembler_.clear_pending(); previous_reached_.reset();
      if (external_mode_ && external_active_) external_result_ = "accepted";
    }
  }
  void consume(const can_frame& f, int64_t received, int64_t wall) {
    if (uart_mode_) {
      publish(raw_pub_, {{"schema", "lift_uart_rx_v1"},
                        {"frame_type", f.can_id == canid_t(0x580 + node_) ? 0x81 : 0x90},
                        {"page", f.can_id == canid_t(0x580 + node_) ? -1 : int((f.can_id - 0x180 - node_) / 0x100)},
                        {"payload_bytes", std::vector<uint8_t>(f.data, f.data + 8)},
                        {"receive_mono_ns", std::to_string(received)},
                        {"host_wall_ns", std::to_string(wall)}, {"timestamp_quality", "host_read_first_byte"}});
    } else publish(raw_pub_, {{"id", f.can_id}, {"dlc", f.can_dlc},
                      {"bytes", std::vector<uint8_t>(f.data, f.data + 8)},
                      {"receive_mono_ns", std::to_string(received)}, {"kernel_wall_ns", std::to_string(wall)}});
    if (f.can_id & CAN_ERR_FLAG) {
      const canid_t classes = f.can_id & CAN_ERR_MASK;
      // A lost arbitration or rejected CAN frame does not by itself mean that
      // application ACKs/telemetry have stopped. Keep those watchdogs running;
      // error notifications must neither refresh them nor discard good state.
      // Controller/transceiver faults, restart, overload, unknown reports and
      // warning-level counters remain fatal. In particular KCAN may report
      // CAN_ERR_CRTL with zero detail bytes when a counter is exactly 96.
      constexpr canid_t frame_errors = CAN_ERR_LOSTARB | CAN_ERR_PROT |
                                        CAN_ERR_ACK | CAN_ERR_BUSERROR;
      const bool fatal = f.can_dlc != CAN_ERR_DLC || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) ||
                         !classes || (classes & ~frame_errors) ||
                         ((classes & CAN_ERR_PROT) && (f.data[2] & CAN_ERR_PROT_OVERLOAD)) ||
                         f.data[6] >= 96 || f.data[7] >= 96;
      ++can_error_frames_;
      if (!fatal) ++can_frame_diagnostics_;
      publish(events_pub_, {{"schema", "lift_can_error_v1"}, {"event", "can_error"},
                           {"action_id", action_id_}, {"error_classes", classes}, {"fatal", fatal},
                           {"bytes", std::vector<uint8_t>(f.data, f.data + 8)},
                           {"receive_mono_ns", std::to_string(received)},
                           {"kernel_wall_ns", std::to_string(wall)}});
      if (fatal) throw std::runtime_error("lift CAN controller/link fault; explicit re-enable required");
      return;
    }
    if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) || f.can_dlc != 8) return;
    if (f.can_id == canid_t(0x580 + node_)) { ack(f, received); return; }
    auto s = assembler_.consume(f, received, uart_mode_ ? 0 : wall);
    if (!s) return;
    sample_ = s;
    if (s->flags & 4) startup_released_ = true;
    const auto t = luxi::mono();
    observe_settling(*s, t);
    if (home_complete_) {
      const bool locked = s->first_ns > home_closed_ns_ && s->healthy(t) && s->stationary() &&
                          !(s->flags & 64) && std::abs(int64_t(s->current_um)) <= 10;
      home_locked_samples_ = locked ? std::min(home_locked_samples_ + 1, 3) : 0;
    }
    std::vector<std::string> times, walls;
    for (int i = 0; i < 4; ++i) { times.push_back(std::to_string(s->page_ns[i])); walls.push_back(std::to_string(s->kernel_wall_ns[i])); }
    publish(state_pub_, {{"schema", "lift_state_v1"}, {"sequence", s->sequence},
                        {"current_um", s->current_um}, {"position_m", s->current_um * 1e-6},
                        {"target_um", s->target_um}, {"speed_rpm", s->speed_rpm},
                        {"target_status", s->target_status}, {"flags", s->flags}, {"fault", s->fault},
                        {"raw_mask", s->raw_mask}, {"stable_mask", s->stable_mask},
                        {"result", s->result}, {"guard", s->guard}, {"uptime_ms", s->uptime_ms},
                        {"feedback_age_ms", s->feedback_age_ms}, {"effective_age_ms", s->age_ms(t)},
                        {"valid", s->valid(t)}, {"page_mono_ns", times}, {"kernel_wall_ns", walls},
                        {"host_wall_ns", std::to_string(wall)},
                        {"timestamp_quality", uart_mode_ ? "host_read_first_byte" : "can_kernel_receive"},
                        {"receive_first_mono_ns", std::to_string(s->first_ns)},
                        {"receive_mono_ns", std::to_string(s->received_ns)},
                        {"ros_time_ns", std::to_string(now().nanoseconds())}});
    if (enabled_ && !link_healthy(t)) throw std::runtime_error(health_reason(t));
    if (phase_ == "moving") {
      if (s->target_status >= 0xe0 || !(s->flags & 64)) throw std::runtime_error("lift motion canceled or session lost");
      bool reached = s->first_ns > ack_time_ && s->valid(t) && (s->flags & 3) == 3 &&
                     s->target_status == 3 && s->target_um == target_ && s->stationary() &&
                     std::abs(int64_t(s->current_um) - target_) <= 10;
      if (!reached) { previous_reached_.reset(); reached_count_ = 0; }
      else {
        if (previous_reached_) {
          uint32_t elapsed = s->uptime_ms - previous_reached_->uptime_ms;
          reached_count_ = s->sequence != previous_reached_->sequence && elapsed > 0 && elapsed <= 300 ? 2 : 1;
        } else reached_count_ = 1;
        previous_reached_ = s;
      }
    }
    if (phase_ == "stopping") {
      if (s->valid(t) && s->stationary() && !(s->flags & 64)) {
        if (stopped_uptime_ >= 0 && uint32_t(s->uptime_ms - uint32_t(stopped_uptime_)) <= 300) ++stopped_count_;
        else stopped_count_ = 1;
        stopped_uptime_ = s->uptime_ms;
        if (stopped_count_ >= 2) { phase_ = "idle"; reason_ = "release_both_grips_then_press"; }
      } else { stopped_count_ = 0; stopped_uptime_ = -1; }
    }
  }
  void tick() {
    const auto previous_feedback = sample_ ? sample_->first_ns : 0;
    try {
      can_frame f{}; int64_t wall = 0;
      for (int i = 0; i < 128; ++i) {
        if (uart_mode_) {
          int64_t received = 0;
          if (!uart_->receive(f, received, wall)) break;
          consume(f, received, wall); continue;
        }
        if (!socket_->receive(f, wall)) break;
        timespec real{}; clock_gettime(CLOCK_REALTIME, &real);
        int64_t t = luxi::mono(), age = int64_t(real.tv_sec) * 1000000000 + real.tv_nsec - wall;
        if (!wall || age < 0) throw std::runtime_error("lift missing/discontinuous CAN receive timestamp");
        if (age > 300000000) throw std::runtime_error("lift CAN receive backlog");
        consume(f, t - age, wall);
      }
      const auto t = luxi::mono();
      if (coordinated_mode_ && !dashboard_position_ && !coordination_.live(t)) {
        if (!coordination_.epoch.empty()) coordination_.blocked = true;
        if (enabled_ || nonce_) cancel("coordinator_lost_or_stopped");
      }
      if (external_mode_ && enabled_ && (!external_.live(t) || menu_ || !link_healthy(t)))
        cancel(!link_healthy(t) ? health_reason(t) : menu_ ? "recorder_failure_menu" : "external_lease_expired");
      if (!external_mode_ && enabled_ && (!input_ok_ || menu_ || !link_healthy(t) ||
          !luxi::fresh(quest_time_, t, coordinated_mode_ ? 500000000 : 200000000))) {
        if (coordinated_mode_) coordination_.blocked = true;
        cancel(!link_healthy(t) ? health_reason(t) : "quest_feedback_lost");
      } else if (!external_mode_ && enabled_ && !luxi::fresh(quest_time_, t, 200000000)) {
        pause_input();
      }
      if (session_) {
        if (heartbeat_ && t - heartbeat_->first > 300000000) throw std::runtime_error("lift heartbeat ACK timeout");
        if (!heartbeat_ && t - last_heartbeat_ >= 100000000) {
          auto hb = lp::request(node_, lp::Keepalive, sequence(), nonce_, uart_mode_);
          heartbeat_ = Pending{hb, t, t, 1, false}; last_heartbeat_ = t;
          transmit(hb, "heartbeat");
        }
      }
      if (pending_) {
        if (t - pending_->first > (uart_mode_ ? 3000000000LL : 2000000000LL)) throw std::runtime_error("lift ACK timeout; outcome unknown, no replay");
        if (!pending_->queued && pending_->attempts < 3 && t - pending_->last >= 100000000 && t - pending_->first < 400000000) {
          transmit(pending_->frame, "identical_retry"); pending_->last = t; ++pending_->attempts;
        }
      }
      if (phase_ == "configuring" && !pending_ && t >= next_info_) request(lp::Info, page_ == 10 ? 25 : page_);
      if (uart_mode_ && enabled_ && !pending_ && t >= next_info_ &&
          (phase_ == "idle" || phase_ == "waiting_for_settle" || phase_ == "session_ready")) request(lp::Info, 25);
      if (phase_ == "waiting_for_settle") {
        if (t > settle_deadline_) throw std::runtime_error("lift did not settle before command");
        if (!pending_ && sample_ && !(sample_->flags & 64) && motion_settled(t)) start_session();
      }
      if (phase_ == "session_ready" && !pending_ && sample_ && sample_->first_ns > session_started_ && (sample_->flags & 64)) next_move();
      if (phase_ == "session_ready" && t - session_started_ > (uart_mode_ ? 5000000000LL : 1000000000LL)) throw std::runtime_error("lift session/READY feedback absent");
      if (phase_ == "moving" && t > move_deadline_) throw std::runtime_error("lift motion timed out");
      if (external_mode_ && external_active_ && t > external_deadline_)
        throw std::runtime_error("external lift task timeout including READY");
      if (phase_ == "moving" && reached_count_ >= 2) {
        if (external_mode_ && external_active_) {
          external_finishing_ = true; external_result_ = "settling";
          cancel("external_arrival_closing_session", true);
        } else if (want_home_) {
          cancel("saved_home_reached_closing_session");
          home_complete_ = true; home_locked_samples_ = 0; home_closed_ns_ = luxi::mono();
        } else cancel("held_target_reached_release_grips", true);
      }
      if (external_mode_ && external_active_ && external_finishing_ && phase_ == "idle" &&
          !pending_ && motion_settled(t) && sample_->target_um == external_target_ &&
          !(sample_->flags & 64) && std::abs(int64_t(sample_->current_um) - external_target_) <= 10) {
        external_result_ = "reached"; external_active_ = external_finishing_ = false;
        reason_ = "external_target_reached_and_READY";
      }
    } catch (const lp::UartDisconnected& e) {
      if (coordinated_mode_) coordination_.blocked = true;
      home_complete_ = false; cancel(e.what()); sample_.reset(); ++run_;
      // node_main releases the descriptor and exits nonzero. The existing
      // systemd Restart=on-failure policy then reopens the configured port.
      // A fresh process starts disabled, with no pending target or nonce;
      // coordinated active messages alone cannot restore motion permission.
      throw;
    } catch (const std::exception& e) {
      if (coordinated_mode_) coordination_.blocked = true;
      home_complete_ = false; cancel(e.what()); sample_.reset(); ++run_;
    }
    // Status carries the coordinator's original feedback timestamp. Publish
    // each new complete snapshot immediately; a separate 100 ms timer alone
    // can leave it checking old metadata after newer CAN feedback arrived.
    // Keep the periodic update for watchdogs/phase changes, without relaxing
    // any feedback-age, heartbeat or controller-fault limits.
    if (++ticks_ % 10 == 0 || (sample_ ? sample_->first_ns : 0) != previous_feedback) {
      publish(status_pub_, {{"enabled", enabled_}, {"phase", phase_}, {"reason", reason_},
                           {"dashboard_position_available", dashboard_available_}, {"dashboard_position", dashboard_position_},
                           {"external_control", external_mode_}, {"external_owner", external_.owner},
                           {"external_task_id", external_task_}, {"external_target_um", external_target_},
                           {"external_result", external_result_},
                           {"external_lease_ms", external_.lease_ns / 1000000},
                           {"external_heartbeat_age_ms", external_.received_ns ? (luxi::mono() - external_.received_ns) / 1000000 : -1},
                           {"external_max_gap_ms", external_.max_gap_ns / 1000000},
                           {"shared_enable", coordinated_mode_ && !dashboard_position_}, {"shared_epoch", coordination_.epoch},
                           {"shared_phase", coordination_.phase}, {"shared_failed", coordination_.blocked},
                           {"shared_ready", coordination_.live(luxi::mono()) && enabled_ && phase_ == "idle" &&
                                            page_ == 10 && command_ready(luxi::mono()) && link_healthy(luxi::mono()) && sample_->stationary() && !(sample_->flags & 64)},
                           {"stationary", link_healthy(luxi::mono()) && sample_->stationary()},
                           {"session_present", !sample_ || bool(sample_->flags & 64)},
                           {"home_complete", home_locked(luxi::mono())},
                           {"feedback_first_mono_ns", std::to_string(sample_ ? sample_->first_ns : 0)},
                           {"feedback_age_ms", sample_ ? int(sample_->feedback_age_ms) : 65535},
                           {"interface", interface_}, {"allow_motion", allow_motion_},
                           {"feedback_fresh", feedback(luxi::mono())}, {"session", session_},
                           {"feedback_healthy", link_healthy(luxi::mono())},
                           {"health", health_reason(luxi::mono())},
                           {"ready_for_command", command_ready(luxi::mono())},
                           {"startup_feedback_grace", startup_grace(luxi::mono())},
                           {"motion_started_mono_ns", std::to_string(motion_started_)},
                           {"startup_origin_um", startup_origin_},
                           {"current_um", sample_ ? sample_->current_um : 0},
                           {"flags", sample_ ? int(sample_->flags) : 0},
                           {"fault", sample_ ? int(sample_->fault) : 0},
                           {"speed_rpm", sample_ ? int(sample_->speed_rpm) : 0},
                           {"control_mode", external_mode_ ? "external_position" : "hold_to_limit"}, {"margin_um", margin_},
                           {"input_recovery", input_recovery_},
                           {"can_error_frames", can_error_frames_},
                           {"can_frame_diagnostics", can_frame_diagnostics_},
                           {"uart_parser_errors", uart_ ? uart_->errors() : 0},
                           {"host_min_um", soft_min_}, {"host_max_um", soft_max_},
                           {"firmware_version", configuration_[1]},
                           {"min_um", configuration_[2]}, {"max_um", configuration_[3]},
                           {"missing_groups", assembler_.missing}, {"duplicate_pages", assembler_.duplicates},
                           {"conflicting_pages", assembler_.conflicts},
                           {"receive_mono_ns", std::to_string(luxi::mono())}});
    }
  }
  void quest(const luxi::Quest& q) {
    if (external_mode_) {
      if (q.buttons[1] || q.buttons[2]) cancel("Quest_B_X_stop");
      return; // No Quest movement or re-enable in external mode.
    }
    const auto t = luxi::mono();
    const bool fresh = luxi::fresh(q.receive_mono_ns, t, 200000000) && q.receive_mono_ns > quest_time_;
    // The first callback after an outage may precede tick(). Handle the old
    // receive timestamp before replacing it, so no queued command can resume.
    if (fresh && enabled_ && coordinated_mode_) {
      if (!luxi::fresh(quest_time_, q.receive_mono_ns, 500000000)) {
        coordination_.blocked = true; cancel("quest_timeout");
      } else if (!luxi::fresh(quest_time_, q.receive_mono_ns, 200000000)) pause_input();
    }
    const bool had_input = input(t);
    input_ok_ = fresh && q.tracking < 2 && q.left_tracking < 2 && q.right_tracking < 2;
    quest_time_ = q.receive_mono_ns;
    for (auto x : q.grips) if (!std::isfinite(x) || x < 0 || x > 1) input_ok_ = false;
    grips_released_ = input_ok_ && q.grips[0] < .35 && q.grips[1] < .35;
    if (!had_input || !input_ok_) neutral_ = false;
    if (!input_ok_ || q.buttons[1] || menu_) {
      if (coordinated_mode_ && (enabled_ || nonce_)) coordination_.blocked = true;
      if (enabled_ || nonce_) cancel(q.buttons[1] ? "B_operator_disable" : "invalid_quest_or_failure_menu");
      left_ = right_ = false; return;
    }
    if (input_recovery_) {
      if (!grips_released_) recovery_neutral_since_ = 0;
      else if (!recovery_neutral_since_) recovery_neutral_since_ = q.receive_mono_ns;
      else if (q.receive_mono_ns - recovery_neutral_since_ >= 100000000) {
        input_recovery_ = false; recovery_neutral_since_ = 0; neutral_ = true;
        event("input_resume", "fresh_released_grips_confirmed");
      }
      left_ = right_ = false;
      return; // A separate new press is required; never replay a held grip.
    }
    if (coordinated_mode_ && (!coordination_.live(t) || coordination_.phase != "active")) return;
    left_ = q.grips[0] >= (left_ ? .35 : .65);
    right_ = q.grips[1] >= (right_ ? .35 : .65);
    const int desired = left_ == right_ ? 0 : (right_ ? 1 : -1);
    const bool released = q.grips[0] < .35 && q.grips[1] < .35;
    if (direction_ && desired != direction_) { cancel("grip_release_both_or_reverse", true); return; }
    if (left_ && right_) { if (enabled_ && phase_ != "stopping") cancel("both_grips_stop", true); neutral_ = false; return; }
    if (!enabled_ || (phase_ != "idle" && phase_ != "stopping")) return;
    // Remember the release during our own stopping interval. A new held press
    // may wait for settling, but never inherits permission from an old hold.
    if (released) neutral_ = true;
    if (desired && neutral_) {
      neutral_ = false;
      if (!link_healthy(t) || (phase_ == "idle" &&
          (!sample_->stationary() || (sample_->flags & 64)))) { cancel("local_motion_or_other_session"); return; }
      direction_ = desired;
      wait_for_settling();
    }
  }

 public:
  Lift() : Node("lift_adapter"), node_(declare_parameter("node_id", 1)),
           uart_mode_(declare_parameter<std::string>("transport", "can") == "uart"), assembler_(node_, uart_mode_) {
    const auto transport = get_parameter("transport").as_string();
    if (transport != "can" && transport != "uart") throw std::invalid_argument("lift transport must be can or uart");
    interface_ = declare_parameter<std::string>("interface", "luxi_lift");
    const auto port = declare_parameter<std::string>("serial_port", "");
    if (uart_mode_) interface_ = port;
    soft_min_ = declare_parameter("host_min_um", uart_mode_ ? -97000 : -100000);
    soft_max_ = declare_parameter("host_max_um", uart_mode_ ? 180000 : 200000);
    allow_motion_ = declare_parameter("allow_motion", false);
    dashboard_available_ = declare_parameter("dashboard_position_control", false);
    coordinated_mode_ = declare_parameter("shared_enable", false);
    external_mode_ = declare_parameter("external_control", false);
    const auto external_lease_ms = declare_parameter("external_lease_ms", 1000);
    if (external_lease_ms < 200 || external_lease_ms > 2000)
      throw std::invalid_argument("external_lease_ms must be between 200 and 2000");
    external_.lease_ns = int64_t(external_lease_ms) * 1000000;
    if (external_mode_ && (coordinated_mode_ || !uart_mode_))
      throw std::runtime_error("external lift control requires UART and excludes shared X");
    // Accept old launch overrides during upgrades; held travel no longer uses steps.
    declare_parameter("step_um", 1000);
    margin_ = declare_parameter("margin_um", 5000);
    if (margin_ < 1000 || margin_ > 50000) throw std::invalid_argument("lift margin");
    if (soft_min_ < (uart_mode_ ? -180000 : -100000) || soft_max_ > (uart_mode_ ? 180000 : 200000) ||
        int64_t(soft_max_) - soft_min_ <= 2 * margin_) throw std::invalid_argument("lift host bounds");
    if (!declare_parameter("allow_hardware", false)) throw std::runtime_error("lift requires explicit hardware selection");
    state_pub_ = create_publisher<std_msgs::msg::String>("/luxi/lift/state", 20);
    raw_pub_ = create_publisher<std_msgs::msg::String>(uart_mode_ ? "/luxi/lift/uart_rx" : "/luxi/lift/can_rx", 100);
    events_pub_ = create_publisher<std_msgs::msg::String>("/luxi/lift/events", 100);
    status_pub_ = create_publisher<std_msgs::msg::String>("/luxi/lift/status", rclcpp::QoS(1).transient_local());
    if (uart_mode_) uart_ = std::make_unique<lp::Uart>(interface_, node_);
    else socket_ = std::make_unique<lp::Socket>(interface_, node_);
    coordination_sub_ = create_subscription<std_msgs::msg::String>("/luxi/coordinated_control", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) { coordinate(m->data); });
    external_sub_ = create_subscription<std_msgs::msg::String>("/luxi/lift/external_command", 10,
        [this](std_msgs::msg::String::ConstSharedPtr m) { external_command(m->data); });
    quest_sub_ = create_subscription<luxi::Quest>("/luxi/quest", 1, [this](luxi::Quest::ConstSharedPtr q) { quest(*q); });
    recorder_sub_ = create_subscription<std_msgs::msg::String>("/luxi/recorder/status", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (!j.is_object()) return;
          menu_ = j.value("failure_menu", false);
          if (menu_ && enabled_) { if (coordinated_mode_) coordination_.blocked = true; cancel("recorder_failure_menu"); }
        });
    control_ = create_service<luxi::Service>("/luxi/lift/control",
        [this](std::shared_ptr<luxi::Service::Request> req, std::shared_ptr<luxi::Service::Response> res) {
          res->success = false;
          if (req->command.rfind("position_", 0) == 0) {
            try {
              const auto t = luxi::mono();
              if (!dashboard_available_ || !allow_motion_ || !uart_mode_)
                throw std::runtime_error("网页位置控制未开启或没有运动许可");
              const auto op = req->command.substr(9);
              if (op != "arm" && op != "heartbeat" && op != "move" && op != "stop")
                throw std::runtime_error("未知网页位置指令");
              auto j = luxi::Json::parse(req->detail);
              if (!j.is_object() || !j.contains("owner") || !j["owner"].is_string() ||
                  !j.contains("sequence") || !j["sequence"].is_number_unsigned())
                throw std::runtime_error("网页控制会话格式错误");
              if (op == "arm") {
                if (enabled_ || external_mode_ || (coordination_.live(t) && coordination_.phase != "halt" && coordination_.phase != "disabled") || !link_healthy(t) ||
                    !sample_->stationary() || (sample_->flags & 64) || menu_)
                  throw std::runtime_error("需要设备健康、停稳且无其他控制会话");
                dashboard_position_ = true; external_mode_ = true;
              } else if (!dashboard_position_ || j["owner"] != external_.owner)
                throw std::runtime_error("网页控制会话已失效，请重新开启");
              j["op"] = op; j["source_mono_ns"] = std::to_string(t);
              external_command(j.dump());
              res->success = op == "stop" || (dashboard_position_ && enabled_ && !external_.revoked);
              res->message = res->success ? "网页位置指令已接收" : reason_;
            } catch (const std::exception& e) { res->message = e.what(); }
            return;
          }
          if (external_mode_ && req->command != "stop" && req->command != "disable") {
            res->message = "external_control: explicit bridge ARM/leased commands only"; return;
          }
          if (coordinated_mode_ && req->command == "start") {
            res->message = "Use shared X or /luxi/control start"; return;
          }
          if (coordinated_mode_ && (req->command == "disable" || req->command == "stop")) coordination_.blocked = true;
          if (req->command == "disable") { cancel("dashboard_disable"); res->success = true; }
          else if (req->command == "stop") {
            cancel("explicit_global_stop");
            try { transmit(lp::request(node_, lp::Stop, sequence(), 0, uart_mode_), "explicit_global_stop"); res->success = true; }
            catch (const std::exception& e) { reason_ = e.what(); }
          } else if (req->command == "start") {
            const auto t = luxi::mono();
            if (allow_motion_ && !enabled_ && input(t) && link_healthy(t) && sample_->stationary() && !(sample_->flags & 64) &&
                grips_released_) {
              enabled_ = true; neutral_ = false; page_ = 0; configuration_.fill(0);
              ready_ = false; ready_ns_ = 0; ready_after_ = t;
              ++run_; phase_ = "configuring"; reason_ = "reading_configuration_no_motion";
              next_info_ = 0; res->success = true;
            } else reason_ = "need motion permission, fresh valid feedback/Quest, stopped lift, no other session, both grips released";
          }
          res->message = reason_;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
  }
  ~Lift() { try { cancel("shutdown"); } catch (...) {} }
};
int main(int argc, char** argv) { return luxi::node_main(argc, argv, [] { return std::make_shared<Lift>(); }); }
