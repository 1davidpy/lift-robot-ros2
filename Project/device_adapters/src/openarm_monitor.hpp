// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace luxi::monitor {
struct Motor {
  std::array<float, 3> range{};
  std::array<bool, 3> range_valid{};
  std::array<uint8_t, 8> data{};
  int64_t seen = 0;
  bool calibrated() const { return range_valid[0] && range_valid[1] && range_valid[2]; }
  bool healthy() const { return (data[0] >> 4) <= 1; }
  double position() const { return ((data[1] << 8) | data[2]) / 65535. * 2 * range[0] - range[0]; }
  double velocity() const { return ((data[3] << 4) | (data[4] >> 4)) / 4095. * 2 * range[1] - range[1]; }
  double torque() const { return (((data[4] & 15) << 8) | data[5]) / 4095. * 2 * range[2] - range[2]; }
};

// Only parameter reads (0x33) and state refreshes (0xCC) are representable.
// In particular, constructing a monitor never invokes the SDK's mode setters.
inline can_frame request(unsigned motor, unsigned rid = 0) {
  if (motor < 1 || motor > 8 || (rid && (rid < 21 || rid > 23)))
    throw std::invalid_argument("read-only motor request");
  can_frame f{};
  f.can_id = 0x7ff;
  f.can_dlc = 8;
  f.data[0] = motor;
  f.data[2] = rid ? 0x33 : 0xcc;
  f.data[3] = rid;
  return f;
}

inline void consume(std::array<Motor, 8>& motors, const canfd_frame& f, int64_t time) {
  if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) || f.len != 8 ||
      f.can_id < 0x11 || f.can_id > 0x18) return;
  const unsigned motor = f.can_id - 0x10;
  auto& m = motors[motor - 1];
  if (f.data[0] == motor && f.data[1] == 0 && f.data[2] == 0x33) {
    if (f.data[3] >= 21 && f.data[3] <= 23) {
      float value;
      std::memcpy(&value, f.data + 4, sizeof(value));
      if (std::isfinite(value) && value > 0 && value < 10000) {
        const unsigned k = f.data[3] - 21;
        m.range[k] = value;
        m.range_valid[k] = true;
      }
    }
    return; // A parameter reply must never mark a joint's state as fresh.
  }
  if ((f.data[0] & 15) != motor) return;
  std::copy_n(f.data, 8, m.data.begin());
  m.seen = time;
}

class Bus {
  int fd_ = -1;
  unsigned index_ = 0, cursor_ = 0, param_cursor_ = 0;
  std::string iface_;
 public:
  std::array<Motor, 8> motors{};
  explicit Bus(const std::string& iface) : iface_(iface) {
    index_ = if_nametoindex(iface.c_str());
    if (!index_) throw std::runtime_error("missing CAN interface: " + iface);
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("monitor CAN socket");
    int on = 1;
    can_filter filter{0x10, CAN_EFF_FLAG | CAN_RTR_FLAG | 0x7f0};
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = index_;
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) ||
        setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) ||
        setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) ||
        bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr))) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("bind monitor CAN: " + iface);
    }
  }
  ~Bus() { if (fd_ >= 0) close(fd_); }
  Bus(const Bus&) = delete;
  Bus& operator=(const Bus&) = delete;
  void tick(int64_t time) {
    if (if_nametoindex(iface_.c_str()) != index_)
      throw std::runtime_error("monitor CAN interface replaced: " + iface_);
    canfd_frame frame{};
    // Consume before requesting, so kernel queues cannot accumulate stale frames.
    for (int n = 0; n < 256; ++n) {
      alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))]{};
      iovec io{&frame, sizeof(frame)};
      msghdr msg{};
      msg.msg_iov = &io; msg.msg_iovlen = 1;
      msg.msg_control = control; msg.msg_controllen = sizeof(control);
      const auto size = recvmsg(fd_, &msg, MSG_DONTWAIT);
      if (size < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        throw std::runtime_error("monitor CAN receive: " + iface_);
      }
      int64_t arrival = 0;
      for (auto* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SO_TIMESTAMPNS ||
            c->cmsg_len < CMSG_LEN(sizeof(timespec))) continue;
        timespec stamp{}, now{};
        std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
        clock_gettime(CLOCK_REALTIME, &now);
        const int64_t age = (int64_t(now.tv_sec) - stamp.tv_sec) * 1000000000 + now.tv_nsec - stamp.tv_nsec;
        if (age >= 0 && age < 100000000) arrival = time - age;
      }
      if (arrival > 0 && (size == CAN_MTU || size == CANFD_MTU)) consume(motors, frame, arrival);
    }
    auto send_read = [this](const can_frame& f) {
      if (send(fd_, &f, sizeof(f), MSG_DONTWAIT) != sizeof(f))
        throw std::runtime_error("monitor CAN read request failed: " + iface_);
    };
    send_read(request(cursor_ + 1));
    cursor_ = (cursor_ + 1) % 8; // 12.5 Hz per joint with a 10 ms node timer.
    // Retry missing ranges slowly; never guess a motor model's decoding limits.
    if (cursor_ == 0) {
      const unsigned motor = param_cursor_ / 3, k = param_cursor_ % 3;
      if (!motors[motor].range_valid[k]) send_read(request(motor + 1, k + 21));
      param_cursor_ = (param_cursor_ + 1) % 24;
    }
  }
};
} // namespace luxi::monitor
