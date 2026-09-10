// SPDX-License-Identifier: Apache-2.0
#include <cstring>
#include <filesystem>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>
int main(int argc, char** argv) {
  try {
    if (argc != 3)
      throw std::runtime_error("usage: socketcan_probe --listen|--virtual-loopback interface");
    std::string mode = argv[1], iface = argv[2];
    if (iface.empty() || iface.find_first_not_of(
                             "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
                             std::string::npos)
      throw std::runtime_error("invalid interface");
    openarm::canbus::CANSocket rx(iface, true);
    if (mode == "--listen") {
      int received = 0;
      // Read-only API validation; never send or configure a physical interface.
      for (int i = 0; i < 20; ++i) {
        if (!rx.is_data_available(100000)) continue;
        canfd_frame frame{};
        auto n = rx.read_raw_frame(&frame, sizeof(frame));
        if (n != CAN_MTU && n != CANFD_MTU) throw std::runtime_error("unexpected frame size");
        ++received;
      }
      std::cout << "SocketCAN FD receive socket OK, observed frames=" << received << '\n';
      return 0;
    }
    if (mode != "--virtual-loopback" || iface != "luxi_test_can" ||
        std::filesystem::canonical("/sys/class/net/" + iface) !=
            "/sys/devices/virtual/net/luxi_test_can")
      throw std::runtime_error(
          "transmit test only permits dedicated virtual interface luxi_test_can");
    openarm::canbus::CANSocket tx(iface, true);
    can_frame classic{};
    classic.can_id = 0x123;
    classic.can_dlc = 8;
    for (int i = 0; i < 8; ++i) classic.data[i] = i;
    if (!tx.write_can_frame(classic) || !rx.is_data_available(1000000))
      throw std::runtime_error("classic CAN transmit/receive");
    can_frame got{};
    if (!rx.read_can_frame(got) || got.can_id != classic.can_id || got.can_dlc != 8 ||
        std::memcmp(got.data, classic.data, 8))
      throw std::runtime_error("classic payload differs");
    canfd_frame fd{};
    fd.can_id = CAN_EFF_FLAG | 0x1234567;
    fd.len = 64;
    fd.flags = CANFD_BRS;
    for (int i = 0; i < 64; ++i) fd.data[i] = 63 - i;
    if (!tx.write_canfd_frame(fd) || !rx.is_data_available(1000000))
      throw std::runtime_error("CAN FD transmit/receive");
    canfd_frame received{};
    if (!rx.read_canfd_frame(received) || received.can_id != fd.can_id || received.len != 64 ||
        std::memcmp(received.data, fd.data, 64))
      throw std::runtime_error("FD payload differs");
    std::cout << "PASS: original OpenArm SocketCAN API, classic 8-byte + extended-ID FD 64-byte "
                 "round trip\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
