#include <luxi/r680.hpp>
#include <luxi/lift_uart.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>

using J = nlohmann::json;
static int64_t mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  J out;
  out["method"] = "Passive reception only; no CAN or UART application commands transmitted";
  try {
    int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd < 0) throw std::runtime_error("CAN socket creation failed");
    sockaddr_can address{}; address.can_family = AF_CAN;
    address.can_ifindex = if_nametoindex("luxi_base");
    if (!address.can_ifindex || bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address))
      throw std::runtime_error("Cannot bind luxi_base");
    luxi::r680::Decoder decoder;
    std::optional<luxi::r680::Feedback> latest;
    auto end = mono() + 1800000000LL;
    while (mono() < end) {
      can_frame frame{};
      if (read(fd, &frame, sizeof frame) == sizeof frame) {
        auto sample = decoder.consume(frame, mono());
        if (sample) latest = *sample;
      } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    close(fd);
    out["base"] = {{"groups", decoder.accepted}, {"rejected", decoder.rejected}, {"responding", latest.has_value()}};
    if (latest) {
      out["base"]["voltage"] = latest->voltage;
      out["base"]["velocity"] = latest->velocity;
      out["base"]["flag_stop"] = latest->stop;
    }
  } catch (const std::exception& e) { out["base"]["error"] = e.what(); }
  try {
    luxi::lift::Uart uart(argv[1], 1);
    luxi::lift::Assembler assembler(1, true);
    std::optional<luxi::lift::Sample> latest;
    unsigned samples = 0;
    auto end = mono() + 2200000000LL;
    while (mono() < end) {
      can_frame frame{}; int64_t t = 0, wall = 0;
      if (uart.receive(frame, t, wall)) {
        auto sample = assembler.consume(frame, t, wall);
        if (sample) { latest = *sample; ++samples; }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    out["lift"] = {{"groups", samples}, {"responding", latest.has_value()},
      {"uart_crc_or_parser_errors", uart.errors()}, {"transport", "uart"}, {"port", argv[1]}};
    if (latest) {
      out["lift"]["current_um"] = latest->current_um;
      out["lift"]["speed_rpm"] = latest->speed_rpm;
      out["lift"]["fault"] = latest->fault;
      out["lift"]["flags"] = latest->flags;
      out["lift"]["feedback_age_ms"] = latest->feedback_age_ms;
      out["lift"]["measurement_valid"] = latest->valid(mono());
      out["lift"]["stationary"] = latest->stationary();
    }
  } catch (const std::exception& e) { out["lift"]["error"] = e.what(); }
  std::ofstream(argv[2]) << out.dump(2);
  std::cout << out.dump(2) << '\n';
}
