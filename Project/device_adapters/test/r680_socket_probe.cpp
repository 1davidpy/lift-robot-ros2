// SPDX-License-Identifier: Apache-2.0
#include <filesystem>
#include <iostream>
#include <luxi/r680.hpp>
#include <luxi/r680_socket.hpp>
int main() {
  int peer = -1;
  try {
    const std::string iface = "luxi_r680_test";
    if (std::filesystem::canonical("/sys/class/net/" + iface) !=
        "/sys/devices/virtual/net/" + iface)
      throw std::runtime_error("dedicated virtual interface required");
    luxi::r680::Socket driver(iface);
    bool exclusive = false;
    try {
      luxi::r680::Socket second(iface);
    } catch (const std::exception&) {
      exclusive = true;
    }
    if (!exclusive) throw std::runtime_error("owner lock");
    peer = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (peer < 0) throw std::runtime_error("peer socket");
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = if_nametoindex(iface.c_str());
    timeval timeout{1, 0};
    setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (bind(peer, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
      throw std::runtime_error("peer bind");
    driver.send(luxi::r680::encode({{.123, -.456, .789}}));
    can_frame received{};
    if (read(peer, &received, sizeof(received)) != sizeof(received) || received.can_id != 0x181 ||
        luxi::r680::be16(received.data + 2) != -456)
      throw std::runtime_error("command roundtrip");
    const uint8_t bytes[24] = {0x7b, 0, 0, 0xc8, 0xff, 0x9c, 1, 0x2c, 0,    0,    0,    0,
                               0x40, 0, 0, 0,    0,    0,    0, 0,    0x5d, 0xc0, 0x20, 0x7d};
    luxi::r680::Decoder decoder;
    std::optional<luxi::r680::Feedback> state;
    for (int i = 0; i < 3; ++i) {
      can_frame f{};
      f.can_id = 0x101 + i;
      f.can_dlc = 8;
      std::copy_n(bytes + i * 8, 8, f.data);
      if (write(peer, &f, sizeof(f)) != sizeof(f)) throw std::runtime_error("feedback send");
      int64_t wall = 0;
      bool got = false;
      for (int attempt = 0; attempt < 100 && !got; ++attempt) {
        got = driver.receive(received, wall);
        if (!got) usleep(1000);
      }
      if (!got || wall <= 0) throw std::runtime_error("feedback kernel timestamp");
      state = decoder.consume(received, 1000000000 + i * 1000000);
    }
    if (!state || state->voltage != 24 || state->velocity[1] != -.1)
      throw std::runtime_error("decoded feedback");
    auto forbidden = luxi::r680::encode({});
    forbidden.can_id = 0x182;
    bool rejected = false;
    try {
      driver.send(forbidden);
    } catch (const std::exception&) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("reserved command ID");
    close(peer);
    std::cout << "PASS: R680 real SocketCAN driver on vcan, command bytes, three-frame feedback, "
                 "kernel timestamps, exclusive owner, reserved-ID rejection\n";
    return 0;
  } catch (const std::exception& e) {
    if (peer >= 0) close(peer);
    std::cerr << e.what() << '\n';
    return 1;
  }
}
