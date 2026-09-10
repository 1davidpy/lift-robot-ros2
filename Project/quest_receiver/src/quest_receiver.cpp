// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <luxi/core.hpp>
#include <std_msgs/msg/string.hpp>
class Receiver : public rclcpp::Node {
  int fd_ = -1;
  uint64_t seq_ = 0;
  double last_t_ = -1;
  int64_t last_rx_ = 0;
  std::string sender_;
  rclcpp::Publisher<luxi::Quest>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  Receiver() : Node("quest_receiver") {
    auto host = declare_parameter("host", "0.0.0.0");
    int port = declare_parameter("port", 5006);
    sender_ = declare_parameter("sender_ip", "");
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) throw std::runtime_error("UDP socket failed");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
        bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(fd_);
      throw std::runtime_error("UDP bind failed");
    }
    pub_ = create_publisher<luxi::Quest>("/luxi/quest", rclcpp::QoS(100).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { receive(); });
  }
  ~Receiver() {
    if (fd_ >= 0) close(fd_);
  }
  void receive() {
    for (int n = 0; n < 256; n++) {
      char buffer[4096];
      sockaddr_in remote{};
      socklen_t size = sizeof(remote);
      auto bytes = recvfrom(fd_, buffer, sizeof(buffer), MSG_TRUNC,
                            reinterpret_cast<sockaddr*>(&remote), &size);
      if (bytes < 0) break;
      if (bytes == 0 || bytes >= 4096) continue;
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &remote.sin_addr, ip, sizeof(ip));
      if (!sender_.empty() && sender_ != ip) continue;
      auto t = luxi::mono();
      try {
        auto q = luxi::parse_quest(std::string(buffer, bytes));
        // Sender clock reset is accepted only after an actual receive timeout.
        if (luxi::fresh(last_rx_, t, 1000000000) && q.sender_time <= last_t_) continue;
        last_t_ = q.sender_time;
        last_rx_ = t;
        q.sequence = ++seq_;
        q.receive_mono_ns = t;
        q.header.stamp = now();
        q.header.frame_id = "quest_world";
        pub_->publish(q);
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected Quest packet: %s",
                             e.what());
      }
    }
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Receiver>(); });
}
