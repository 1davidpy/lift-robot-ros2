#include <net/if.h>
// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <fcntl.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <luxi/core.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <std_msgs/msg/string.hpp>
#include "openarm_monitor.hpp"
namespace oa = openarm::can::socket;
namespace dm = openarm::damiao_motor;
class Adapter : public rclcpp::Node {
  luxi::Json json_;
  luxi::ArmConfig cfg_;
  bool monitor_only_ = false;
  bool startup_allowed_ = false, startup_owned_ = false, startup_pending_ = false;
  bool startup_recover_ = false;
  std::array<double, 16> startup_pose_{};
  std::array<double, 16> startup_reference_{};
  std::array<double, 16> startup_nominal_{}, startup_bias_{};
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
    startup_phase_ = "hold";
    startup_reason_ = reason;
    startup_arrived_ = 0;
    // Retain the submitted target and its supporting torque. Replacing it
    // with measured position would unload the proportional controller.
  }
  bool startup_command(const std::string& command, std::string& message, const std::string& detail = "") {
    const auto t = luxi::mono();
    if (!startup_allowed_) { message = "启动姿态功能未配置"; return false; }
    if (command == "disable") {
      if (enabled_ && !startup_owned_) { message = "请通过遥操作控制撤销使能"; return false; }
      if (startup_owned_ && enabled_)
        for (auto& a : arms_) if (a) a->disable_all();
      enabled_ = startup_owned_ = startup_pending_ = false;
      startup_phase_ = "idle"; startup_reason_ = "operator_disable";
      message = "已撤销抬臂使能并移除转矩"; return true;
    }
    if (command == "stop") {
      if (startup_owned_) startup_hold("operator_stop");
      message = "已停止抬臂，保留当前位置转矩"; return true;
    }
    const bool restore = command == "restore_hold";
    if (command != "start" && !restore) { message = "不支持的启动指令"; return false; }
    if (startup_phase_ == "moving" || startup_pending_ || (enabled_ && !startup_owned_)) {
      message = "双臂正在运动或已由遥操作控制"; return false;
    }
    if (control_status_.is_object() && luxi::fresh(control_status_time_, t, 500000000) &&
        control_status_.value("phase", "disabled") != "disabled" &&
        control_status_.value("system_phase", "disabled") != "stopped") {
      message = "请先停止遥操作"; return false;
    }
    if (!luxi::valid_state(state_) || !luxi::fresh(state_.receive_mono_ns, t)) {
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
      const auto values = luxi::Json::parse(detail).get<std::vector<double>>();
      if (values.size() != 16) { message = "保持恢复需要 16 轴目标"; return false; }
      for (int i = 0; i < 16; ++i) {
        if (!std::isfinite(values[i]) || values[i] < cfg_.lo[i] || values[i] > cfg_.hi[i] ||
            std::abs(values[i] - state_.position[i]) > .2) {
          message = "保持目标与当前位置不一致或超出限位"; return false;
        }
        restore_target_[i] = values[i];
      }
    }
    // Mode writes happen only after the explicit button, never on read-only boot.
    startup_reference_ = state_.position;
    if (monitor_only_) {
      if (!mock_) {
        for (auto& m : monitors_) m.reset();
        for (int side = 0; side < 2; ++side) initialize_arm(side);
      }
      monitor_only_ = false;
      seen_.fill(0);
    }
    startup_owned_ = true; startup_pending_ = !enabled_;
    restore_hold_ = restore;
    if (!enabled_) { sent_ = state_.position; startup_bias_.fill(0); }
    for (int i = 0; i < 16; ++i) startup_nominal_[i] = sent_[i] - startup_bias_[i];
    startup_since_ = t; startup_arrived_ = 0; startup_recover_ = false;
    startup_phase_ = startup_pending_ ? "preparing" : restore ? "hold" : "moving";
    startup_reason_ = restore ? "calibration_hold_restored" : "operator_start";
    if (restore && !startup_pending_) sent_ = restore_target_;
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
      sent_ = state_.position;
      startup_nominal_ = state_.position;
      enabled_ = true; // Ensure unwinding disables even a partially enabled pair.
      for (auto& a : arms_) if (a) a->enable_all();
      startup_pending_ = false;
      startup_phase_ = restore_hold_ ? "hold" : "moving"; startup_since_ = t;
      if (restore_hold_) sent_ = restore_target_;
    }
    if (!enabled_) return;
    if (!healthy) {
      startup_phase_ = "hold"; startup_reason_ = "feedback_lost";
      startup_recover_ = true; return;
    }
    if (startup_recover_) { sent_ = state_.position; startup_recover_ = false; }
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
        bool nominal_arrived = true;
        for (int i = 0; i < 16; ++i) {
          const double step = std::min(cfg_.speed[i], startup_speed_) * dt;
          startup_nominal_[i] += std::clamp(startup_pose_[i] - startup_nominal_[i], -step, step);
          if (std::abs(startup_nominal_[i] - startup_pose_[i]) > .0001) nominal_arrived = false;
        }
        for (int i = 0; i < 16; ++i) {
          // Slowly remove static load error after the nominal trajectory.
          // Bias is bounded to 0.2 rad, its rate to 0.03 rad/s; grippers
          // keep their position/force mode without integral compensation.
          if (nominal_arrived && i % 8 != 7)
            startup_bias_[i] = std::clamp(startup_bias_[i] +
                std::clamp(.5 * (startup_pose_[i] - state_.position[i]) * dt,
                           -.03 * dt, .03 * dt), -.2, .2);
          const double desired = std::clamp(startup_nominal_[i] + startup_bias_[i], cfg_.lo[i], cfg_.hi[i]);
          const double step = std::min(cfg_.speed[i], startup_speed_) * dt;
          sent_[i] += std::clamp(desired - sent_[i], -step, step);
        }
        if (error <= startup_tolerance_ && speed < .08) {
          if (!startup_arrived_) startup_arrived_ = t;
          if (t - startup_arrived_ >= 500000000) {
            startup_phase_ = "ready"; startup_reason_ = "measured_arrival";
          }
        } else startup_arrived_ = 0;
      }
    }
    command_.position = sent_;
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
    sent_ = state_.position;
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
        "/luxi/command", 1, [this](luxi::Command::ConstSharedPtr c) { if (!startup_owned_) command_ = *c; });
    startup_status_ = create_publisher<std_msgs::msg::String>("/luxi/arms/startup_status", 1);
    control_status_sub_ = create_subscription<std_msgs::msg::String>("/luxi/control_status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (j.is_object()) { control_status_ = j; control_status_time_ = luxi::mono(); }
        });
    startup_service_ = create_service<luxi::Service>("/luxi/arms/startup",
        [this](const std::shared_ptr<luxi::Service::Request> req,
               std::shared_ptr<luxi::Service::Response> res) {
          try { res->success = startup_command(req->command, res->message, req->detail);
            startup_rejection_ = res->success ? "" : res->message; }
          catch (const std::exception& e) {
            startup_hold("initialization_failed"); res->success = false; res->message = e.what();
          }
        });
    life_ = create_subscription<std_msgs::msg::String>(
        "/luxi/driver_lifecycle", 10, [this](std_msgs::msg::String::ConstSharedPtr s) {
          if (monitor_only_) return;
          try {
            if (s->data == "enable" && luxi::valid_state(state_) &&
                luxi::fresh(state_.receive_mono_ns, luxi::mono())) {
              if (startup_pending_ || (startup_owned_ && startup_phase_ == "moving")) return;
              startup_owned_ = false; startup_phase_ = "idle";
              startup_reason_ = "teleop_handover";
              sent_ = state_.position;
              for (auto& a : arms_)
                if (a) a->enable_all();
              enabled_ = true;
            } else if (s->data == "disable") {
              startup_owned_ = startup_pending_ = false;
              startup_phase_ = "idle"; startup_reason_ = "operator_disable";
              for (auto& a : arms_)
                if (a) a->disable_all();
              enabled_ = false;
            }
          } catch (const std::exception& e) {
            enabled_ = false;
            for (auto& a : arms_)
              if (a) try {
                  a->disable_all();
                } catch (...) {
                }
            error(e.what());
          }
        });
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
    double dt = previous_ ? std::min(.05, (t - previous_) * 1e-9) : .01;
    previous_ = t;
    try {
      if (mock_) {
        for (int i = 0; i < 16; i++) {
          double d = enabled_ ? std::clamp(sent_[i] - state_.position[i], -cfg_.speed[i] * dt,
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
      const auto previous_sent = sent_;
      update_startup(t, dt, luxi::valid_state(state_));
      state_.phase = monitor_only_ ? "monitor_only" : (enabled_ ? command_.phase : "disabled");
      if (enabled_ && luxi::valid_state(state_) && luxi::fresh(command_.source_mono_ns, t)) {
        auto target = cfg_.limit(command_.position, sent_, dt);
        if (startup_owned_) {
          // A stopped out-of-limit recovery must hold, never jump to the limit.
          target = command_.position;
          if (startup_phase_ == "moving")
            for (int i = 0; i < 16; ++i) {
              const double desired = std::clamp(command_.position[i], cfg_.lo[i], cfg_.hi[i]);
              const double step = std::min(cfg_.speed[i], startup_speed_) * dt;
              target[i] = previous_sent[i] + std::clamp(desired - previous_sent[i], -step, step);
            }
        }
        if (!mock_)
          for (int s = 0; s < 2; s++) {
            std::vector<dm::MITParam> params;
            for (int i = 0; i < 7; i++) {
              int k = s * 8 + i;
              params.push_back(
                  dm::MITParam{cfg_.kp[k], cfg_.kd[k], target[k] + cfg_.offset[k], 0, 0});
            }
            arms_[s]->get_arm().mit_control_all(params);
            arms_[s]->get_gripper().set_position(target[s * 8 + 7] + cfg_.offset[s * 8 + 7], 50.,
                                                 1. / 4.5);
          }
        sent_ = target;
        auto applied = command_;
        applied.position = sent_;
        applied.header.stamp = now();
        applied.source_mono_ns = luxi::mono();
        applied_->publish(applied);
      } else if (enabled_)
        state_.phase = "hold_feedback_or_command_timeout";
    } catch (const std::exception& e) {
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
    double max_error = 0;
    for (int i = 0; i < 16; ++i)
      max_error = std::max(max_error, std::abs(startup_pose_[i] - state_.position[i]));
    std_msgs::msg::String status;
    status.data = luxi::Json{{"configured", startup_allowed_}, {"owned", startup_owned_},
        {"last_rejection", startup_rejection_}, {"recovery_margin_rad", recovery_margin_},
        {"gripper_recovery_margin_rad", gripper_recovery_margin_},
        {"phase", startup_phase_}, {"reason", startup_reason_}, {"target", startup_pose_},
        {"speed_rad_s", startup_speed_}, {"tolerance_rad", startup_tolerance_},
        {"static_compensation_rad", startup_bias_},
        {"max_error_rad", max_error}, {"receive_mono_ns", std::to_string(luxi::mono())}}.dump();
    startup_status_->publish(status);
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Adapter>(); });
}
