// SPDX-License-Identifier: Apache-2.0
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <luxi/core.hpp>
#include <luxi/r680.hpp>
#include <luxi/r680_socket.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
namespace rb = luxi::r680;
class R680 : public rclcpp::Node {
  bool mock_;
  std::unique_ptr<rb::Socket> socket_;
  rb::Decoder decoder_;
  rb::Gate gate_;
  rb::Velocity sent_{};
  int ticks_ = 0;
  uint64_t command_seq_ = 0;
  int64_t last_tick_ = 0;
  double linear_accel_, angular_accel_;
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
    gate_.feedback_time = s.mono_ns[2];
    gate_.stop = s.stop;
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
      if (ticks_ % 2 == 0) {
        auto wanted = gate_.command(luxi::mono());
        double norm = std::hypot(wanted[0], wanted[1]);
        if (norm > gate_.mapping.linear) {
          wanted[0] *= gate_.mapping.linear / norm;
          wanted[1] *= gate_.mapping.linear / norm;
        }
        if (gate_.reason == "teleop")
          for (int i = 0; i < 3; ++i)
            wanted[i] = sent_[i] + std::clamp(wanted[i] - sent_[i],
                                              -(i == 2 ? angular_accel_ : linear_accel_) * dt * 2,
                                              (i == 2 ? angular_accel_ : linear_accel_) * dt * 2);
        auto frame = rb::encode(wanted);
        if (socket_) socket_->send(frame);
        for (int i = 0; i < 3; ++i) sent_[i] = rb::be16(frame.data + 2 * i) / 1000.0;
        json(command_, {{"schema", "r680_command_v1"},
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
      gate_.halt("can_or_input_fault");
      sent_ = {};
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
    if (ticks_ % 10 == 0)
      json(status_, {{"enabled", gate_.enabled},
                     {"reason", gate_.reason},
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
    bool hardware = declare_parameter("allow_hardware", false);
    gate_.mapping.linear = declare_parameter("max_linear", .2);
    gate_.mapping.angular = declare_parameter("max_angular", .3141592653589793);
    gate_.mapping.deadzone = declare_parameter("deadzone", .16);
    gate_.mapping.reverse_steering = declare_parameter("ps2_reverse_steering", true);
    linear_accel_ = declare_parameter("linear_acceleration", .3);
    angular_accel_ = declare_parameter("angular_acceleration", .6);
    for (double x : {gate_.mapping.linear, gate_.mapping.angular, linear_accel_, angular_accel_})
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
    quest_ =
        create_subscription<luxi::Quest>("/luxi/quest", 1, [this](luxi::Quest::ConstSharedPtr q) {
          gate_.quest_time = q->receive_mono_ns;
          gate_.tracking = q->tracking == 0 && q->left_tracking == 0 && q->right_tracking == 0;
          gate_.sticks = q->sticks;
          gate_.grip = q->grips[0];
          if (q->buttons[1] || q->buttons[2] || q->buttons[3]) gate_.halt("operator_stop");
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
          if (req->command == "start")
            res->success = gate_.start(luxi::mono());
          else if (req->command == "stop" || req->command == "disable") {
            gate_.halt("operator_stop");
            res->success = true;
          }
          res->message =
              res->success
                  ? gate_.reason
                  : "Need fresh feedback/Quest, no menu, centered sticks and released left grip";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
  }
  ~R680() {
    gate_.halt("shutdown");
    if (socket_)
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
