// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <fcntl.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
namespace luxi::r680 {
class Socket {
  int fd_ = -1, lock_ = -1;

 public:
  explicit Socket(const std::string& iface) {
    if (iface.empty() || iface.size() >= IFNAMSIZ ||
        iface.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
            std::string::npos)
      throw std::invalid_argument("CAN interface");
    try {
      lock_ = open(("/tmp/luxi-" + iface + ".lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
      if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) < 0)
        throw std::runtime_error("CAN owner active");
      fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
      if (fd_ < 0) throw std::runtime_error("SocketCAN open");
      ifreq req{};
      std::strncpy(req.ifr_name, iface.c_str(), IFNAMSIZ - 1);
      if (ioctl(fd_, SIOCGIFFLAGS, &req) < 0 || !(req.ifr_flags & IFF_UP))
        throw std::runtime_error("CAN must be UP");
      if (ioctl(fd_, SIOCGIFMTU, &req) < 0 || req.ifr_mtu != CAN_MTU)
        throw std::runtime_error("R680 requires classic CAN MTU 16 (fd off)");
      if (ioctl(fd_, SIOCGIFINDEX, &req) < 0) throw std::runtime_error("CAN index");
      sockaddr_can addr{};
      addr.can_family = AF_CAN;
      addr.can_ifindex = req.ifr_ifindex;
      can_filter filters[3]{};
      for (int i = 0; i < 3; ++i)
        filters[i] = {static_cast<canid_t>(0x101 + i), CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
      can_err_mask_t errors = CAN_ERR_MASK;
      int one = 1;
      if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0 ||
          setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0 ||
          setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one)) < 0 ||
          bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("SocketCAN configure");
    } catch (...) {
      if (fd_ >= 0) close(fd_);
      if (lock_ >= 0) close(lock_);
      throw;
    }
  }
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  ~Socket() {
    if (fd_ >= 0) close(fd_);
    if (lock_ >= 0) close(lock_);
  }
  void send(const can_frame& f) {
    if (f.can_id != 0x181 || f.can_dlc != 8 || f.data[6] || f.data[7])
      throw std::runtime_error("R680 transmit contract");
    if (write(fd_, &f, sizeof(f)) != sizeof(f))
      throw std::runtime_error(std::string("R680 CAN transmit: ") + strerror(errno));
  }
  bool receive(can_frame& f, int64_t& wall_ns) {
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))]{};
    iovec io{&f, sizeof(f)};
    msghdr msg{};
    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    auto n = recvmsg(fd_, &msg, MSG_DONTWAIT);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
    if (n != sizeof(f) || (msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)))
      throw std::runtime_error("R680 CAN receive");
    wall_ns = 0;
    for (auto* h = CMSG_FIRSTHDR(&msg); h; h = CMSG_NXTHDR(&msg, h))
      if (h->cmsg_level == SOL_SOCKET && h->cmsg_type == SCM_TIMESTAMPNS) {
        timespec ts{};
        std::memcpy(&ts, CMSG_DATA(h), sizeof(ts));
        wall_ns = int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
      }
    return true;
  }
};
}  // namespace luxi::r680
