// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <luxi/lift_protocol.hpp>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <ctime>
#include <deque>
#include <string>
#include <vector>

namespace luxi::lift {
// An unplugged USB descriptor never becomes usable again, even if Linux
// reuses its ttyUSB number. The owner must release it and reopen explicitly.
class UartDisconnected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
inline uint16_t uart_crc(const uint8_t* data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}
inline std::vector<uint8_t> uart_frame(uint8_t type, const uint8_t* data, size_t size) {
  if (!type || size > 16) throw std::invalid_argument("UART frame");
  std::vector<uint8_t> out(size + 6);
  out[0] = 0xa5; out[1] = 0x5a; out[2] = uint8_t(size + 1); out[3] = type;
  if (size) std::copy_n(data, size, out.data() + 4);
  const auto crc = uart_crc(out.data() + 2, size + 2);
  out[size + 4] = crc & 255; out[size + 5] = crc >> 8;
  return out;
}
struct UartPacket {
  uint8_t type = 0;
  std::vector<uint8_t> data;
  int64_t first_ns = 0, wall_ns = 0;
};
class UartParser {
  struct Byte { uint8_t value; int64_t mono, wall; };
  std::deque<Byte> bytes_;
 public:
  uint64_t errors = 0;
  void clear() { bytes_.clear(); }
  void feed(const uint8_t* data, size_t size, int64_t mono, int64_t wall) {
    if (bytes_.size() + size > 8192) { clear(); ++errors; throw std::runtime_error("UART receive buffer overflow"); }
    for (size_t i = 0; i < size; ++i) bytes_.push_back({data[i], mono, wall});
  }
  bool next(UartPacket& out) {
    while (bytes_.size() >= 3) {
      if (bytes_[0].value != 0xa5 || bytes_[1].value != 0x5a) { bytes_.pop_front(); continue; }
      const size_t length = bytes_[2].value;
      if (length < 1 || length > 17) { bytes_.pop_front(); ++errors; continue; }
      const size_t total = length + 5;
      if (bytes_.size() < total) return false;
      std::array<uint8_t, 22> frame{};
      for (size_t i = 0; i < total; ++i) frame[i] = bytes_[i].value;
      if (uart_crc(frame.data() + 2, total - 4) != u16(frame.data() + total - 2)) {
        bytes_.pop_front(); ++errors; continue; // Resynchronise, including an embedded next SOF.
      }
      out.type = frame[3]; out.first_ns = bytes_[0].mono; out.wall_ns = bytes_[0].wall;
      out.data.assign(frame.begin() + 4, frame.begin() + total - 2);
      for (size_t i = 0; i < total; ++i) bytes_.pop_front();
      if (!((out.type == 0x81 && out.data.size() == 8) ||
            (out.type == 0x90 && out.data.size() == 9 && out.data[0] < 4))) { ++errors; continue; }
      return true;
    }
    return false;
  }
};
class Uart {
  int fd_ = -1, node_;
  int64_t last_poll_ = 0;
  UartParser parser_;
  static int64_t clock_ns(clockid_t clock) {
    timespec ts{}; clock_gettime(clock, &ts);
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
  }
 public:
  explicit Uart(const std::string& path, int node) : node_(node) {
    if (path.rfind("/dev/", 0) != 0) throw std::invalid_argument("explicit UART device path required");
    fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error(std::string("UART open: ") + strerror(errno));
    try {
      if (flock(fd_, LOCK_EX | LOCK_NB) < 0 || ioctl(fd_, TIOCEXCL) < 0)
        throw std::runtime_error("UART port already owned or exclusive access unavailable");
      termios cfg{};
      if (tcgetattr(fd_, &cfg) < 0) throw std::runtime_error("UART termios read");
      cfmakeraw(&cfg);
      cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
      cfg.c_cflag |= CS8 | CLOCAL | CREAD;
      cfg.c_iflag &= ~(IXON | IXOFF | IXANY);
      cfg.c_cc[VMIN] = 0; cfg.c_cc[VTIME] = 0;
      cfsetispeed(&cfg, B115200); cfsetospeed(&cfg, B115200);
      if (tcsetattr(fd_, TCSANOW, &cfg) < 0) throw std::runtime_error("UART configure 115200 8N1");
      int lines = TIOCM_DTR | TIOCM_RTS;
      if (ioctl(fd_, TIOCMBIC, &lines) < 0 && errno != ENOTTY)
        throw std::runtime_error("UART DTR/RTS configuration");
      if (tcflush(fd_, TCIFLUSH) < 0) throw std::runtime_error("UART initial receive flush");
      last_poll_ = clock_ns(CLOCK_MONOTONIC);
    } catch (...) { ioctl(fd_, TIOCNXCL); close(fd_); fd_ = -1; throw; }
  }
  Uart(const Uart&) = delete;
  Uart& operator=(const Uart&) = delete;
  ~Uart() { if (fd_ >= 0) { ioctl(fd_, TIOCNXCL); close(fd_); } }
  uint64_t errors() const { return parser_.errors; }
  void send(const can_frame& payload) {
    // can_frame is only the existing internal eight-byte message container.
    // No CAN interface is opened and no CAN identifier is transmitted on UART.
    if (payload.can_dlc != 8 || payload.data[6] || payload.data[7] != 1)
      throw std::runtime_error("UART request payload contract");
    const auto bytes = uart_frame(1, payload.data, 8);
    const auto deadline = clock_ns(CLOCK_MONOTONIC) + 20000000;
    size_t offset = 0;
    while (offset < bytes.size()) {
      if (clock_ns(CLOCK_MONOTONIC) >= deadline) throw std::runtime_error("UART write timeout; outcome unknown");
      const auto n = write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (n > 0) { offset += size_t(n); continue; }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd p{fd_, POLLOUT, 0}; poll(&p, 1, 1); continue;
      }
      throw UartDisconnected(std::string("UART write: ") + strerror(errno));
    }
  }
  bool receive(can_frame& message, int64_t& mono, int64_t& wall) {
    const auto now = clock_ns(CLOCK_MONOTONIC);
    const bool stalled = now - last_poll_ > 300000000;
    last_poll_ = now;
    if (stalled) {
      parser_.clear(); tcflush(fd_, TCIFLUSH);
      throw std::runtime_error("UART polling stalled; discarded un-timestamped backlog");
    }
    pollfd p{fd_, POLLIN, 0};
    if (poll(&p, 1, 0) < 0 && errno != EINTR) throw UartDisconnected("UART poll failed");
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw UartDisconnected("UART USB disconnected");
    UartPacket packet;
    if (!parser_.next(packet)) {
      uint8_t buffer[1024];
      const auto n = read(fd_, buffer, sizeof(buffer));
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        throw UartDisconnected(std::string("UART read: ") + strerror(errno));
      if (n > 0) parser_.feed(buffer, size_t(n), now, clock_ns(CLOCK_REALTIME));
      if (!parser_.next(packet)) return false;
    }
    mono = packet.first_ns; wall = packet.wall_ns;
    if (now - mono > 300000000) throw std::runtime_error("UART frame assembly timeout");
    message = {}; message.can_dlc = 8;
    if (packet.type == 0x81) {
      message.can_id = 0x580 + node_;
      std::copy_n(packet.data.begin(), 8, message.data);
    } else {
      message.can_id = 0x180 + node_ + 0x100 * packet.data[0];
      std::copy_n(packet.data.begin() + 1, 8, message.data);
    }
    return true;
  }
};
}  // namespace luxi::lift
