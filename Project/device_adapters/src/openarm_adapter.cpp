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
namespace oa = openarm::can::socket;
namespace dm = openarm::damiao_motor;
class Adapter : public rclcpp::Node {
  luxi::Json json_;
  luxi::ArmConfig cfg_;
  bool mock_, enabled_ = false;
  std::array<std::unique_ptr<oa::OpenArm>, 2> arms_;
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
      seen_[side * 8 + id - 0x11] = luxi::mono();
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
    state_.position = cfg_.home;
    for (int i = 0; i < 16; i++)
      state_.position[i] = std::clamp(state_.position[i], cfg_.lo[i], cfg_.hi[i]);
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
        arms_[s] = std::make_unique<oa::OpenArm>(iface, json_.at("fd"));
        arms_[s]->init_arm_motors(
            {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340,
             dm::MotorType::DM4340, dm::MotorType::DM4310, dm::MotorType::DM4310,
             dm::MotorType::DM4310},
            {1, 2, 3, 4, 5, 6, 7}, {17, 18, 19, 20, 21, 22, 23});
        arms_[s]->init_gripper_motor(dm::MotorType::DM4310, 8, 24, dm::ControlMode::POS_FORCE);
        arms_[s]->set_callback_mode_all(dm::CallbackMode::STATE);
      }
    }
    pub_ = create_publisher<luxi::State>("/luxi/arms/state", 5);
    applied_ = create_publisher<luxi::Command>("/luxi/applied", 10);
    errors_ = create_publisher<std_msgs::msg::String>("/luxi/faults", 20);
    sub_ = create_subscription<luxi::Command>(
        "/luxi/command", 1, [this](luxi::Command::ConstSharedPtr c) { command_ = *c; });
    life_ = create_subscription<std_msgs::msg::String>(
        "/luxi/driver_lifecycle", 10, [this](std_msgs::msg::String::ConstSharedPtr s) {
          try {
            if (s->data == "enable" && luxi::valid_state(state_) &&
                luxi::fresh(state_.receive_mono_ns, luxi::mono())) {
              sent_ = state_.position;
              for (auto& a : arms_)
                if (a) a->enable_all();
              enabled_ = true;
            } else if (s->data == "disable") {
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
      } else
        for (int s = 0; s < 2; s++) {
          arms_[s]->refresh_all();
          receive(s);
        }
      auto feedback_now = luxi::mono();
      for (int i = 0; i < 16; i++) {
        state_.age_ns[i] = seen_[i] ? feedback_now - seen_[i] : INT64_MAX;
        state_.valid[i] = luxi::fresh(seen_[i], feedback_now) && std::isfinite(state_.position[i]);
      }
      state_.phase = command_.phase;
      if (enabled_ && luxi::valid_state(state_) && luxi::fresh(command_.source_mono_ns, t)) {
        auto target = cfg_.limit(command_.position, sent_, dt);
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
    }
    state_.enabled = enabled_;
    state_.header.stamp = now();
    state_.header.frame_id = "arm_origin";
    state_.receive_mono_ns = luxi::mono();
    pub_->publish(state_);
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Adapter>(); });
}
