// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <arpa/inet.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <ctime>
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
  luxi::Quest pending_;
  bool pending_valid_ = false, pending_stop_ = false;
  rclcpp::Publisher<luxi::Quest>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  Receiver() : Node("quest_receiver") {
    auto host = declare_parameter("host", "0.0.0.0");
    int port = declare_parameter("port", 5006);
    sender_ = declare_parameter("sender_ip", "");
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) throw std::runtime_error("UDP socket failed");
    int timestamps = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &timestamps, sizeof(timestamps)) < 0)
      throw std::runtime_error("UDP kernel timestamp setup failed");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
        bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(fd_);
      throw std::runtime_error("UDP bind failed");
    }
    pub_ = create_publisher<luxi::Quest>("/luxi/quest", rclcpp::QoS(1).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { receive(); });
  }
  ~Receiver() {
    if (fd_ >= 0) close(fd_);
  }
  void receive() {
    for (int n = 0; n < 256; n++) {
      char buffer[4096];
      sockaddr_in remote{};
      alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))]{};
      iovec data{buffer, sizeof(buffer)};
      msghdr message{};
      message.msg_name = &remote; message.msg_namelen = sizeof(remote);
      message.msg_iov = &data; message.msg_iovlen = 1;
      message.msg_control = control; message.msg_controllen = sizeof(control);
      auto bytes = recvmsg(fd_, &message, MSG_TRUNC);
      if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          if (pending_valid_ && luxi::fresh(pending_.receive_mono_ns, luxi::mono())) {
            pending_.buttons[1] = pending_.buttons[1] || pending_stop_;
            pub_->publish(pending_);
          }
          pending_valid_ = pending_stop_ = false;
        }
        return;
      }
      if (bytes == 0 || bytes >= 4096) continue;
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &remote.sin_addr, ip, sizeof(ip));
      if (!sender_.empty() && sender_ != ip) continue;
      const auto t = luxi::mono();
      int64_t arrival = 0;
      timespec wall{}; clock_gettime(CLOCK_REALTIME, &wall);
      for (auto* c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS &&
            c->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
          timespec stamp{}; std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
          const auto age = (wall.tv_sec - stamp.tv_sec) * 1000000000LL + wall.tv_nsec - stamp.tv_nsec;
          if (age >= 0) arrival = t - age;
        }
      }
      // Socket backlog must not acquire a new lease when the process wakes up.
      if (!luxi::fresh(arrival, t)) continue;
      try {
        auto q = luxi::parse_quest(std::string(buffer, bytes));
        // Sender clock reset is accepted only after an actual receive timeout.
        if (luxi::fresh(last_rx_, t, 11000000000LL) && q.sender_time <= last_t_) continue;
        last_t_ = q.sender_time;
        last_rx_ = arrival;
        q.sequence = ++seq_;
        q.receive_mono_ns = arrival;
        q.header.stamp = now();
        q.header.frame_id = "quest_world";
        pending_stop_ = pending_stop_ || q.buttons[1];
        pending_ = std::move(q); pending_valid_ = true;
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
