// SPDX-License-Identifier: Apache-2.0
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <luxi/core.hpp>
#include <luxi/external_control.hpp>
#include <luxi/coordinated_control.hpp>
#include <luxi/r680.hpp>
#include <luxi/r680_socket.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
#include <deque>
#include <set>
namespace rb = luxi::r680;
class R680 : public rclcpp::Node {
  bool mock_;
  bool external_mode_ = false;
  luxi::ExternalControl external_;
  rb::Velocity external_velocity_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr external_sub_;
  void external_command(const std::string& data) {
    if (!external_mode_) return;
    try {
      const auto t = luxi::mono();
      const auto j = luxi::Json::parse(data);
      const auto op = external_.accept(j, t);
      if (op == "stop") {
        external_.revoke(); external_velocity_ = {}; gate_.halt("external_stop");
      } else if (op == "arm") {
        if (receive_only_ || gate_.enabled || gate_.menu || gate_.stop || stationary_samples_ < 2 ||
            !rb::fresh(gate_.feedback_time, t, 100000000))
          throw std::runtime_error("external ARM requires fresh stopped chassis");
        external_velocity_ = {}; gate_.enabled = true; gate_.reason = "external_ready";
      } else if (op == "velocity") {
        if (!gate_.enabled || !j.contains("velocity") || !j["velocity"].is_array() || j["velocity"].size() != 3)
          throw std::runtime_error("external velocity unavailable");
        rb::Velocity value{};
        for (int i = 0; i < 3; ++i) {
          if (!j["velocity"][i].is_number()) throw std::runtime_error("invalid velocity scalar");
          value[i] = j["velocity"][i].get<double>();
          if (!std::isfinite(value[i])) throw std::runtime_error("nonfinite velocity");
        }
        if (std::hypot(value[0], value[1]) > .10 || std::abs(value[2]) > .20)
          throw std::runtime_error("external velocity exceeds commissioning limits");
        external_velocity_ = value;
      } else throw std::runtime_error("unsupported external base operation");
    } catch (const std::exception& e) {
      external_.revoke(); external_velocity_ = {}; gate_.halt(e.what());
    }
  }
  bool receive_only_ = false;
  bool keyboard_ = false;
  std::string keyboard_owner_;
  bool web_keyboard_paused_ = false;
  uint64_t web_keyboard_sequence_ = 0;
  std::string web_keyboard_pause_reason_, last_keyboard_release_reason_;
  static constexpr int64_t web_keyboard_timeout_ns_ = 500000000;
  void pause_web_keyboard(const std::string& reason) {
    if (!web_keyboard_paused_) {
      RCLCPP_WARN(get_logger(), "Web base motion paused, ownership retained: %s", reason.c_str());
      web_keyboard_pause_reason_ = reason;
    }
    web_keyboard_paused_ = true;
    keyboard_velocity_ = {}; sent_ = {}; gate_.reason = web_keyboard_pause_reason_;
  }
  std::set<std::string> keyboard_owners_;
  std::deque<std::string> keyboard_owner_order_;
  double web_keyboard_linear_limit_ = .05;
  bool keyboard_coordinator_idle(int64_t t) const {
    return !coordinated_mode_ ||
        (luxi::fresh(coordination_.sent_ns, t, 250000000) &&
         coordination_.control_source != "teleop" &&
         (coordination_.phase == "disabled" || coordination_.phase == "halt"));
  }
  void release_keyboard(const std::string& reason) {
    if (!keyboard_owner_.empty()) {
      last_keyboard_release_reason_ = reason;
      RCLCPP_INFO(get_logger(), "Web base ownership released: %s", reason.c_str());
    }
    web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear(); web_keyboard_sequence_ = 0;
    keyboard_ = navigation_ = false;
    keyboard_owner_.clear(); keyboard_velocity_ = {}; keyboard_time_ = 0;
    sent_ = {};
    gate_.halt(reason);
  }
  bool web_keyboard_command(const std::string& command, const std::string& detail,
                            bool& success, std::string& message) {
    const bool scoped_stop = command == "stop" && !detail.empty();
    const bool start = command == "keyboard_start";
    const bool update = command == "keyboard";
    if (!scoped_stop && !start && !update) return false;
    if (!coordinated_mode_ && keyboard_owner_.empty() && detail.empty()) return false;
    // Preserve legacy array commands only for independent, unowned sessions.
    if (update && !coordinated_mode_ && keyboard_owner_.empty() &&
        !detail.empty() && detail.front() == '[') return false;
    success = false;
    if (scoped_stop) {
      if (keyboard_ && (detail == keyboard_owner_ || detail == "__web_operator_release__"))
        release_keyboard("web_keyboard_released");
      success = true; message = "Web control released; stale owners cannot stop a new controller";
      return true;
    }
    try {
      const auto t = luxi::mono();
      if (start) {
        if (detail.empty() || detail.size() > 96) throw std::invalid_argument("Web control owner required");
        if (keyboard_ && gate_.enabled && detail == keyboard_owner_) {
          success = true; message = "Web control already accepted"; return true;
        }
        if (keyboard_owners_.count(detail)) throw std::invalid_argument("Expired web control; turn the switch on again");
        if (receive_only_ || gate_.enabled || !keyboard_coordinator_idle(t) || gate_.menu || gate_.stop ||
            stationary_samples_ < 2 || !rb::fresh(gate_.feedback_time, t, 100000000))
          throw std::invalid_argument("Base busy or not ready; stop Quest control before enabling web control");
        keyboard_owners_.insert(detail); keyboard_owner_order_.push_back(detail);
        if (keyboard_owner_order_.size() > 256) {
          keyboard_owners_.erase(keyboard_owner_order_.front()); keyboard_owner_order_.pop_front();
        }
        keyboard_owner_ = detail; keyboard_ = true; navigation_ = false;
        web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear(); web_keyboard_sequence_ = 0;
        keyboard_velocity_ = {}; keyboard_time_ = t; sent_ = {};
        gate_.enabled = true; gate_.reason = "web_keyboard_ready";
        success = true; message = "Web base control acquired"; return true;
      }
      const auto j = luxi::Json::parse(detail);
      if (!j.is_object() || !j.contains("owner") || !j.at("owner").is_string() ||
          j.at("owner").get<std::string>() != keyboard_owner_ || keyboard_owner_.empty() ||
          !keyboard_ || navigation_ || !gate_.enabled)
        throw std::invalid_argument("Web control expired or owned by another controller");
      if (!j.contains("sequence") || !j.at("sequence").is_number_unsigned())
        throw std::invalid_argument("Web command sequence required; refresh the console");
      const auto sequence = j.at("sequence").get<uint64_t>();
      if (sequence <= web_keyboard_sequence_)
        throw std::invalid_argument("Stale web command sequence");
      const auto v = j.at("velocity").get<std::array<double, 3>>();
      for (double value : v)
        if (!std::isfinite(value) || std::abs(value) > 1) throw std::invalid_argument("Invalid web velocity");
      // Check the old deadline inside the callback as well as the timer.
      // A queued nonzero request must not resume motion after a scheduling gap.
      if (!rb::fresh(keyboard_time_, t, web_keyboard_timeout_ns_))
        pause_web_keyboard("web_command_timeout");
      if (coordinated_mode_ && !luxi::fresh(coordination_.sent_ns, t, 250000000))
        pause_web_keyboard("coordinator_feedback_lost");
      web_keyboard_sequence_ = sequence;
      keyboard_time_ = t;
      if (web_keyboard_paused_) {
        if (v == rb::Velocity{} && rb::fresh(gate_.feedback_time, t, 100000000) &&
            (!coordinated_mode_ || luxi::fresh(coordination_.sent_ns, t, 250000000))) {
          web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear();
        } else {
          success = true; message = "Web motion paused; release WASD to continue"; return true;
        }
      }
      const double scale = std::min(web_keyboard_linear_limit_, gate_.mapping.linear) /
          std::max(1., std::hypot(v[0], v[1]));
      keyboard_velocity_ = {v[0] * scale, v[1] * scale, v[2] * .30};
      keyboard_time_ = t; success = true; message = "Web command accepted";
    } catch (const std::exception& error) {
      // A rejected or late request cannot release a different owner's hold.
      message = error.what();
    }
    return true;
  }
  bool navigation_ = false;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  bool y_released_ = false;
  bool coordinated_mode_ = false;
  luxi::CoordinatedControl coordination_;
  int stationary_samples_ = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr coordination_sub_;
  int64_t keyboard_time_ = 0;
  rb::Velocity keyboard_velocity_{};
  std::string interface_;
  std::unique_ptr<rb::Socket> socket_;
  rb::Decoder decoder_;
  rb::Gate gate_;
  rb::Velocity sent_{};
  int ticks_ = 0;
  uint64_t command_seq_ = 0;
  int64_t last_tick_ = 0;
  double linear_accel_, angular_accel_;
  double manual_linear_accel_, manual_angular_accel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_, raw_, command_, status_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Subscription<luxi::Quest>::SharedPtr quest_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr menu_;
  rclcpp::Service<luxi::Service>::SharedPtr control_;
  rclcpp::TimerBase::SharedPtr timer_;
  void json(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& p, const luxi::Json& value) {
    std_msgs::msg::String m;
    m.data = value.dump();
    p->publish(m);
  }
  bool shared_ready(int64_t t) const {
    auto candidate = gate_;
    candidate.enabled = false;
    return !keyboard_ && !receive_only_ && stationary_samples_ >= 2 && candidate.start(t);
  }
  void coordinate(const std::string& data) {
    if (!coordinated_mode_) return;
    const auto t = luxi::mono();
    const bool changed = coordination_.accept(data, t);
    if (keyboard_ && gate_.enabled) {
      // Incoming Quest packets and rejected prepare attempts never steal the
      // web lease. Explicit shared parking remains a stop action.
      if (coordination_.phase == "stop" || coordination_.phase == "park" ||
          (changed && coordination_.phase == "halt"))
        release_keyboard("shared_operator_stop");
      if (coordination_.phase == "prepare" || coordination_.phase == "active")
        coordination_.blocked = true;
      return;
    }
    if (!coordination_.live(t)) { gate_.halt("coordinator_not_ready"); return; }
    if (!changed) return;
    keyboard_ = false; keyboard_velocity_ = {};
    gate_.halt("shared_" + coordination_.phase);
    if (coordination_.phase == "active" && (!shared_ready(t) || !gate_.start(t))) {
      coordination_.blocked = true; gate_.halt("shared_enable_rejected");
    }
  }
  void consume(const can_frame& frame, int64_t receive_mono, int64_t kernel_wall = 0) {
    json(raw_, {{"id", frame.can_id},
                {"dlc", frame.can_dlc},
                {"bytes", std::vector<uint8_t>(frame.data, frame.data + 8)},
                {"receive_mono_ns", std::to_string(receive_mono)},
                {"kernel_wall_ns", std::to_string(kernel_wall)},
                {"mock", mock_}});
    if (frame.can_id & CAN_ERR_FLAG) {
      gate_.halt("can_error");
      return;
    }
    auto feedback = decoder_.consume(frame, receive_mono);
    if (!feedback) return;
    const auto& s = *feedback;
    if (!rb::fresh(gate_.feedback_time, s.mono_ns[2], 250000000)) stationary_samples_ = 0;
    gate_.feedback_time = s.mono_ns[2];
    gate_.stop = s.stop;
    const bool stopped = std::abs(s.velocity[0]) <= .005 && std::abs(s.velocity[1]) <= .005 &&
                         std::abs(s.velocity[2]) <= .01;
    stationary_samples_ = stopped ? std::min(stationary_samples_ + 1, 3) : 0;
    if (s.stop) gate_.halt("chassis_disabled");
    auto stamp = now();
    json(state_, {{"schema", "r680_v1"},
                  {"velocity", s.velocity},
                  {"acceleration", s.acceleration},
                  {"gyro", s.gyro},
                  {"voltage", s.voltage},
                  {"flag_stop", s.stop},
                  {"frame_mono_ns", std::array<std::string, 3>{{std::to_string(s.mono_ns[0]),
                                                                std::to_string(s.mono_ns[1]),
                                                                std::to_string(s.mono_ns[2])}}},
                  {"receive_mono_ns", std::to_string(s.mono_ns[2])},
                  {"ros_time_ns", std::to_string(stamp.nanoseconds())},
                  {"valid", rb::fresh(s.mono_ns[2], luxi::mono(), 250000000)},
                  {"mock", mock_}});
    geometry_msgs::msg::TwistStamped v;
    v.header.stamp = stamp;
    v.header.frame_id = "base_link";
    v.twist.linear.x = s.velocity[0];
    v.twist.linear.y = s.velocity[1];
    v.twist.angular.z = s.velocity[2];
    velocity_->publish(v);
    sensor_msgs::msg::Imu im;
    im.header = v.header;
    im.orientation_covariance[0] = -1;
    im.linear_acceleration.x = s.acceleration[0];
    im.linear_acceleration.y = s.acceleration[1];
    im.linear_acceleration.z = s.acceleration[2];
    im.angular_velocity.x = s.gyro[0];
    im.angular_velocity.y = s.gyro[1];
    im.angular_velocity.z = s.gyro[2];
    imu_->publish(im);
    sensor_msgs::msg::BatteryState battery;
    battery.header = v.header;
    battery.voltage = s.voltage;
    battery.current = battery.charge = battery.capacity = battery.design_capacity =
        battery.percentage = battery.temperature = std::numeric_limits<float>::quiet_NaN();
    battery.present = true;
    battery_->publish(battery);
  }
  void mock_feedback(int64_t t) {
    std::array<uint8_t, 24> packet{};
    packet[0] = 0x7b;
    packet[23] = 0x7d;
    for (int i = 0; i < 3; ++i) rb::put16(packet.data() + 2 + i * 2, sent_[i], 1000);
    rb::put16(packet.data() + 12, 9.8, 1671.84);
    rb::put16(packet.data() + 20, 24, 1000);
    for (int i = 0; i < 22; ++i) packet[22] ^= packet[i];
    for (int i = 0; i < 3; ++i) {
      can_frame f{};
      f.can_id = 0x101 + i;
      f.can_dlc = 8;
      std::copy_n(packet.data() + 8 * i, 8, f.data);
      consume(f, t);
    }
  }
  void tick() {
    auto t = luxi::mono();
    double dt = last_tick_ ? std::clamp((t - last_tick_) * 1e-9, 0.0, .05) : .02;
    last_tick_ = t;
    try {
      if (mock_) {
        if (ticks_ % 5 == 0) mock_feedback(t);
      } else {
        can_frame f{};
        int64_t wall = 0;
        for (int i = 0; i < 256 && socket_->receive(f, wall); ++i) {
          auto mono = luxi::mono();
          timespec real{};
          clock_gettime(CLOCK_REALTIME, &real);
          int64_t wall_now = int64_t(real.tv_sec) * 1000000000 + real.tv_nsec;
          // Preserve kernel arrival age; do not refresh old queued frames by dequeue time.
          int64_t age = wall ? wall_now - wall : 0;
          if (age < 0) {
            gate_.halt("clock_discontinuity");
            continue;
          }
          consume(f, mono - age, wall);
        }
      }
      // 10 ms receive loop, 20 ms command period, 50 ms feedback mock period.
      if (coordinated_mode_ && keyboard_) {
        if (!luxi::fresh(coordination_.sent_ns, luxi::mono(), 250000000)) {
          if (!keyboard_owner_.empty()) pause_web_keyboard("coordinator_feedback_lost");
          else release_keyboard("coordinator_feedback_lost");
        }
      } else if (coordinated_mode_) {
        if (!coordination_.live(luxi::mono())) {
          if (!coordination_.epoch.empty()) coordination_.blocked = true;
          gate_.halt("coordinator_lost_or_stopped");
        } else if (coordination_.phase != "active") gate_.halt("shared_" + coordination_.phase);
        else if (!gate_.enabled) coordination_.blocked = true;
      }
      if (ticks_ % 2 == 0) {
        rb::Velocity wanted{};
        if (external_mode_) {
          if (!external_.live(luxi::mono()) || gate_.menu || gate_.stop ||
              !rb::fresh(gate_.feedback_time, luxi::mono(), 250000000)) {
            external_.revoke(); gate_.halt("external_lease_or_feedback_lost");
          }
          if (gate_.enabled && rb::fresh(gate_.feedback_time, luxi::mono(), 100000000)) {
            wanted = external_velocity_;
            gate_.reason = wanted == rb::Velocity{} ? "external_ready" : "teleop";
          }
          if (!gate_.enabled) external_.revoke();
        } else if (keyboard_) {
          if (!keyboard_owner_.empty()) {
            if (!rb::fresh(keyboard_time_, luxi::mono(), web_keyboard_timeout_ns_))
              pause_web_keyboard("web_command_timeout");
          } else if (!rb::fresh(keyboard_time_, luxi::mono(), 200000000)) gate_.halt("keyboard_timeout");
          if (!rb::fresh(gate_.feedback_time, luxi::mono(), 250000000)) gate_.halt("feedback_timeout");
          if (gate_.menu || gate_.stop) gate_.halt("chassis_disabled_or_menu");
          if (gate_.enabled && rb::fresh(gate_.feedback_time, luxi::mono(), 100000000)) {
            wanted = web_keyboard_paused_ ? rb::Velocity{} : keyboard_velocity_;
            gate_.reason = web_keyboard_paused_ ? web_keyboard_pause_reason_ :
                wanted == rb::Velocity{} ? "keyboard_ready" : "teleop";
          } else if (gate_.enabled) gate_.reason = "feedback_delayed";
        } else wanted = gate_.command(luxi::mono());
        if (keyboard_ && !gate_.enabled) release_keyboard(gate_.reason);
        if (coordinated_mode_ && coordination_.phase == "active" && !gate_.enabled)
          coordination_.blocked = true;
        double norm = std::hypot(wanted[0], wanted[1]);
        if (norm > gate_.mapping.linear) {
          wanted[0] *= gate_.mapping.linear / norm;
          wanted[1] *= gate_.mapping.linear / norm;
        }
        // Manual buttons use a shorter, bounded ramp; Quest and navigation
        // retain their independently configured acceleration profile.
        const bool manual = keyboard_ && !navigation_;
        const double accel_linear = manual ? manual_linear_accel_ : linear_accel_;
        const double accel_angular = manual ? manual_angular_accel_ : angular_accel_;
        if (gate_.reason == "teleop")
          for (int i = 0; i < 3; ++i)
            wanted[i] = sent_[i] + std::clamp(wanted[i] - sent_[i],
                                              -(i == 2 ? accel_angular : accel_linear) * dt * 2,
                                              (i == 2 ? accel_angular : accel_linear) * dt * 2);
        auto frame = rb::encode(wanted);
        if (socket_ && !receive_only_) socket_->send(frame);
        for (int i = 0; i < 3; ++i) sent_[i] = rb::be16(frame.data + 2 * i) / 1000.0;
        if (!receive_only_) json(command_, {{"schema", "r680_command_v1"},
                        {"sequence", ++command_seq_},
                        {"velocity", sent_},
                        {"source_quest_mono_ns", std::to_string(gate_.quest_time)},
                        {"tx_mono_ns", std::to_string(luxi::mono())},
                        {"bytes", std::vector<uint8_t>(frame.data, frame.data + 8)},
                        {"enabled", gate_.enabled},
                        {"reason", gate_.reason},
                        {"mock", mock_}});
      }
    } catch (const std::exception& e) {
      if (coordinated_mode_) coordination_.blocked = true;
      gate_.halt("can_or_input_fault");
      sent_ = {};
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
      if (!mock_) throw; // Unwind, release CAN sockets; systemd retries without enabling motors.
    }
    if (ticks_ % 10 == 0)
      json(status_, {{"enabled", gate_.enabled},
                     {"external_control", external_mode_}, {"external_owner", external_.owner},
                     {"shared_enable", coordinated_mode_}, {"shared_epoch", coordination_.epoch},
                     {"shared_phase", coordination_.phase}, {"shared_failed", coordination_.blocked},
                     {"shared_ready", coordination_.live(luxi::mono()) && shared_ready(luxi::mono())},
                     {"stationary", stationary_samples_ >= 2 && rb::fresh(gate_.feedback_time, luxi::mono(), 250000000)},
                     {"receive_mono_ns", std::to_string(luxi::mono())},
                     {"feedback_mono_ns", std::to_string(gate_.feedback_time)},
                     {"interface", interface_},
                     {"receive_only", receive_only_},
                     {"input_source", external_mode_ ? "external" : (keyboard_ ? "keyboard" : "quest")},
                     {"navigation_cmd_vel", !external_mode_ && !coordinated_mode_ && !receive_only_},
                     {"navigation_active", navigation_ && gate_.enabled},
                     {"manual_twist", true},
                      {"web_keyboard_control", true},
                      {"web_keyboard_manual_release", true},
                      {"web_keyboard_paused", web_keyboard_paused_},
                      {"web_keyboard_pause_reason", web_keyboard_pause_reason_},
                      {"web_keyboard_command_timeout_ms", web_keyboard_timeout_ns_ / 1000000},
                      {"web_keyboard_sequence", web_keyboard_sequence_},
                      {"last_keyboard_release_reason", last_keyboard_release_reason_},
                      {"web_keyboard_linear_limit", std::min(web_keyboard_linear_limit_, gate_.mapping.linear)},
                      {"keyboard_owner", keyboard_owner_},
                      {"keyboard_takeover_available", keyboard_coordinator_idle(luxi::mono()) && !gate_.enabled},
                      {"control_source", coordination_.control_source},
                     {"manual_linear_limit", std::min(.50, gate_.mapping.linear)},
                     {"manual_angular_limit", .30},
                     {"manual_linear_acceleration", manual_linear_accel_},
                     {"manual_angular_acceleration", manual_angular_accel_},
                     {"require_left_grip", gate_.require_left_grip},
                     {"reason", gate_.reason},
                     {"input_recovery", gate_.input_recovery},
                     {"feedback_fresh", rb::fresh(gate_.feedback_time, luxi::mono(), 250000000)},
                     {"flag_stop", gate_.stop},
                     {"accepted_groups", decoder_.accepted},
                     {"rejected_groups", decoder_.rejected},
                     {"mock", mock_}});
    ++ticks_;
  }

 public:
  R680() : Node("r680_adapter"), mock_(declare_parameter("mock", true)) {
    const auto iface = declare_parameter<std::string>("interface", "luxi_base");
    interface_ = iface;
    receive_only_ = declare_parameter("receive_only", false);
    coordinated_mode_ = declare_parameter("shared_enable", false);
    external_mode_ = declare_parameter("external_control", false);
    if (external_mode_ && (coordinated_mode_ || receive_only_))
      throw std::runtime_error("external_control excludes shared/receive_only modes");
    gate_.recover_short_gaps = coordinated_mode_;
    const auto network_timeout = declare_parameter<double>("network_recovery_timeout", 10.);
    if (!std::isfinite(network_timeout) || network_timeout < .5 || network_timeout > 10.)
      throw std::invalid_argument("network_recovery_timeout must be in [0.5, 10] seconds");
    gate_.network_recovery_ns = static_cast<int64_t>(network_timeout * 1e9);
    if (receive_only_) gate_.reason = "receive_only_no_motion";
    bool hardware = declare_parameter("allow_hardware", false);
    gate_.mapping.linear = declare_parameter("max_linear", .2);
    web_keyboard_linear_limit_ = declare_parameter("web_keyboard_linear_limit", .05);
    if (!std::isfinite(web_keyboard_linear_limit_) || web_keyboard_linear_limit_ <= 0 ||
        web_keyboard_linear_limit_ > .50)
      throw std::invalid_argument("web_keyboard_linear_limit must be in (0, 0.50] m/s");
    gate_.mapping.angular = declare_parameter("max_angular", .3141592653589793);
    gate_.mapping.deadzone = declare_parameter("deadzone", .16);
    gate_.mapping.reverse_steering = declare_parameter("ps2_reverse_steering", true);
    gate_.require_left_grip = declare_parameter("require_left_grip", true);
    linear_accel_ = declare_parameter("linear_acceleration", .3);
    angular_accel_ = declare_parameter("angular_acceleration", .6);
    manual_linear_accel_ = declare_parameter("manual_linear_acceleration", 1.0);
    manual_angular_accel_ = declare_parameter("manual_angular_acceleration", 1.5);
    for (double x : {gate_.mapping.linear, gate_.mapping.angular, linear_accel_, angular_accel_, manual_linear_accel_, manual_angular_accel_})
      if (!std::isfinite(x) || x <= 0 || x > 5)
        throw std::invalid_argument("R680 speed/acceleration configuration");
    if (!std::isfinite(gate_.mapping.deadzone) || gate_.mapping.deadzone < 0 ||
        gate_.mapping.deadzone >= 1)
      throw std::invalid_argument("deadzone");
    if (!mock_) {
      if (!hardware) throw std::runtime_error("R680 hardware requires allow_hardware");
      socket_ = std::make_unique<rb::Socket>(iface);
    }
    state_ = create_publisher<std_msgs::msg::String>("/luxi/base/state", 20);
    raw_ = create_publisher<std_msgs::msg::String>("/luxi/base/can_rx", 100);
    command_ = create_publisher<std_msgs::msg::String>("/luxi/base/applied", 20);
    status_ = create_publisher<std_msgs::msg::String>("/luxi/base/status",
                                                      rclcpp::QoS(1).transient_local());
    velocity_ = create_publisher<geometry_msgs::msg::TwistStamped>("/base/velocity", 10);
    imu_ = create_publisher<sensor_msgs::msg::Imu>("/base/imu", 10);
    battery_ = create_publisher<sensor_msgs::msg::BatteryState>("/base/battery", 10);
    coordination_sub_ = create_subscription<std_msgs::msg::String>("/luxi/coordinated_control", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) { coordinate(m->data); });
    external_sub_ = create_subscription<std_msgs::msg::String>("/luxi/base/external_command", 10,
        [this](std_msgs::msg::String::ConstSharedPtr m) { external_command(m->data); });
    quest_ =
        create_subscription<luxi::Quest>("/luxi/quest", 1, [this](luxi::Quest::ConstSharedPtr q) {
          if (external_mode_) {
            if (q->buttons[1] || q->buttons[2]) {
              external_.revoke(); gate_.halt("Quest_B_X_stop");
            }
            return; // Quest cannot start or steer an external-control driver.
          }
          const auto t = luxi::mono();
          const bool input_fresh = rb::fresh(q->receive_mono_ns, t, 200000000);
          const bool input_ordered = q->receive_mono_ns > gate_.quest_time;
          // Startup, an input gap or reordered data require a new release before
          // Y may enable the base. A held button must never re-enable on recovery.
          if (!input_fresh || !rb::fresh(gate_.quest_time, t, 200000000) ||
              !input_ordered)
            y_released_ = false;
          const bool y_toggle = input_fresh && q->buttons[3] && y_released_;
          if (input_fresh && input_ordered && !keyboard_) gate_.note_input_arrival(q->receive_mono_ns);
          gate_.quest_time = q->receive_mono_ns;
          // Ordinary STALE poses still carry current stick/button input, as in
          // arm teleop. INVALID (including a missing controller pose) must stop.
          gate_.tracking = q->tracking < 2 && q->left_tracking < 2 && q->right_tracking < 2;
          gate_.sticks = q->sticks;
          gate_.grip = q->grips[0];
          y_released_ = input_fresh && input_ordered && gate_.tracking && !q->buttons[3];
          // X belongs only to the arms. B stops the base and overrides Y.
          if (q->buttons[1]) {
            gate_.halt("operator_stop");
            if (coordinated_mode_) coordination_.blocked = true;
          } else if (y_toggle && !coordinated_mode_ && !keyboard_) {
            if (gate_.enabled) {
              gate_.halt("operator_stop");
            } else if (receive_only_) {
              gate_.reason = "receive_only_no_motion";
            } else if (gate_.start(t)) {
              keyboard_ = false;
            } else {
              // Consume rejected presses; centering while holding Y cannot arm.
              gate_.reason = "y_enable_rejected";
            }
          }
        });
    menu_ = create_subscription<std_msgs::msg::String>(
        "/luxi/recorder/status", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (!j.is_discarded()) gate_.menu = j.value("failure_menu", false);
        });
    control_ = create_service<luxi::Service>(
        "/luxi/base/control", [this](const std::shared_ptr<luxi::Service::Request> req,
                                     std::shared_ptr<luxi::Service::Response> res) {
          if (external_mode_) {
            res->success = req->command == "stop" || req->command == "disable";
            if (res->success) { external_.revoke(); gate_.halt("operator_stop"); }
            res->message = "external_control: explicit bridge ARM/leased commands only";
            return;
          }
          if (web_keyboard_command(req->command, req->detail, res->success, res->message)) return;
          if (coordinated_mode_) {
            if (req->command == "stop" || req->command == "disable") {
              coordination_.blocked = true; gate_.halt("operator_stop"); res->success = true;
            }
            res->message = "Shared X control: use /luxi/control start, park or halt; Y/independent enables are inactive";
            return;
          }
          if (receive_only_ && req->command != "stop" && req->command != "disable") {
            res->success = false; res->message = "CAN receive-only diagnostics; motion disabled"; return;
          }
          if (req->command == "keyboard_start" || req->command == "navigation_start") {
            if (!gate_.enabled && !gate_.menu && !gate_.stop &&
                rb::fresh(gate_.feedback_time, luxi::mono(), 100000000)) {
              keyboard_ = true;
              navigation_ = req->command == "navigation_start";
              keyboard_velocity_ = {};
              keyboard_time_ = luxi::mono();
              gate_.enabled = true;
              gate_.reason = "keyboard_ready";
              res->success = true;
            }
          } else if (req->command == "keyboard") {
            auto j = luxi::Json::parse(req->detail, nullptr, false);
            if (keyboard_ && !navigation_ && gate_.enabled && rb::fresh(keyboard_time_, luxi::mono(), 200000000) &&
                j.is_array() && (j.size() == 2 || j.size() == 3) && j[0].is_number() && j[1].is_number() &&
                (j.size() == 2 || j[2].is_number())) {
              double x = j[0].get<double>(), y = j[1].get<double>();
              const double yaw = j.size() == 3 ? j[2].get<double>() : 0.0;
              if (std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
                  std::abs(x) <= 1 && std::abs(y) <= 1 && std::abs(yaw) <= 1) {
                double scale = .50 / std::max(1.0, std::hypot(x, y));
                keyboard_velocity_ = {x * scale, y * scale, yaw * .30};
                keyboard_time_ = luxi::mono();
                res->success = true;
              }
            }
            if (!res->success) gate_.halt("invalid_or_expired_keyboard");
          } else if (req->command == "start") {
            if (!gate_.enabled) {
              res->success = gate_.start(luxi::mono());
              if (res->success) keyboard_ = false;
            }
          }
          else if (req->command == "stop" || req->command == "disable") {
            gate_.halt("operator_stop");
            res->success = true;
          }
          res->message =
              res->success
                  ? gate_.reason
                  : (gate_.require_left_grip
                         ? "Need fresh feedback/Quest, valid tracking, no menu, centered sticks and released left grip"
                         : "Need fresh feedback/Quest, valid tracking, no menu and centered sticks");
        });
    // Atlas supplies base-frame Twist only after an explicit navigation_start.
    // Fresh feedback, the physical stop and the existing 200 ms input watchdog
    // remain enforced by tick(); reconnecting a publisher never arms the base.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS().keep_last(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          if (!navigation_ || !keyboard_ || !gate_.enabled || external_mode_ || coordinated_mode_ || receive_only_) return;
          const double x=msg->linear.x, y=msg->linear.y, yaw=msg->angular.z;
          if (!rb::fresh(keyboard_time_, luxi::mono(), 200000000) ||
              !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
              std::hypot(x,y) > .150001 || std::abs(yaw) > .300001 ||
              msg->linear.z != 0 || msg->angular.x != 0 || msg->angular.y != 0) {
            gate_.halt("invalid_or_expired_cmd_vel"); return;
          }
          keyboard_velocity_ = {x,y,yaw};
          keyboard_time_ = luxi::mono();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
  }
  ~R680() {
    gate_.halt("shutdown");
    if (socket_ && !receive_only_)
      for (int i = 0; i < 10; ++i) {
        try {
          socket_->send(rb::encode({}));
        } catch (...) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<R680>(); });
}
