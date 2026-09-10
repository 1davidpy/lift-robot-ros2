// SPDX-License-Identifier: Apache-2.0
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <luxi/core.hpp>
#include <mutex>
#include <thread>
using namespace std::chrono_literals;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("r680_integration_probe");
  auto client = node->create_client<luxi::Service>("/luxi/base/control");
  auto menu = node->create_publisher<std_msgs::msg::String>("/luxi/recorder/status",
                                                            rclcpp::QoS(1).transient_local());
  std::mutex mutex;
  luxi::Json last, status, feedback;
  auto applied = node->create_subscription<std_msgs::msg::String>(
      "/luxi/base/applied", 20, [&](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mutex);
        last = luxi::Json::parse(m->data);
      });
  auto states = node->create_subscription<std_msgs::msg::String>(
      "/luxi/base/state", 20, [&](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mutex);
        feedback = luxi::Json::parse(m->data);
      });
  auto statuses = node->create_subscription<std_msgs::msg::String>(
      "/luxi/base/status", rclcpp::QoS(1).transient_local(),
      [&](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mutex);
        status = luxi::Json::parse(m->data);
      });
  std::atomic<bool> running{true}, sending{true};
  std::atomic<double> sx{0}, sy{0}, rx{0}, grip{0};
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(15007);
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  std::thread spin([&] { rclcpp::spin(node); });
  std::thread sender([&] {
    uint64_t n = 0;
    while (running) {
      if (sending) {
        luxi::Json pose = {{"x", 0},  {"y", 0},  {"z", 0}, {"qx", 0},
                           {"qy", 0}, {"qz", 0}, {"qw", 1}};
        luxi::Json packet = {{"t", ++n / 72.0},  {"lc", pose},       {"rc", pose},
                             {"lt", 0},          {"rt", 0},          {"lg", grip.load()},
                             {"lsx", sx.load()}, {"lsy", sy.load()}, {"rsx", rx.load()},
                             {"v", 0},           {"vl", 0},          {"vr", 0}};
        auto s = packet.dump();
        sendto(fd, s.data(), s.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
      }
      std::this_thread::sleep_for(13889us);
    }
  });
  auto check = [](bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
  };
  auto wait = [&](auto pred, const char* reason) {
    for (int i = 0; i < 100; ++i) {
      std::this_thread::sleep_for(50ms);
      std::lock_guard<std::mutex> l(mutex);
      if (pred()) return;
    }
    throw std::runtime_error(reason);
  };
  auto request = [&](const char* cmd) {
    check(client->wait_for_service(5s), "base service missing");
    auto req = std::make_shared<luxi::Service::Request>();
    req->command = cmd;
    auto f = client->async_send_request(req);
    check(f.wait_for(5s) == std::future_status::ready, "service timeout");
    return f.get()->success;
  };
  int result = 1;
  try {
    wait([&] { return !status.is_null() && !feedback.is_null() && !last.is_null(); },
         "node startup");
    {
      std::lock_guard<std::mutex> l(mutex);
      check(!status.at("enabled"), "default disabled");
      check(feedback.at("velocity") == luxi::Json::array({0., 0., 0.}), "default zero feedback");
    }
    std::this_thread::sleep_for(300ms);
    check(request("start"), "neutral enable rejected");
    sy = 1;
    std::this_thread::sleep_for(200ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      check(last.at("velocity")[0] == 0, "grip released must remain stopped");
    }
    grip = 1;
    wait([&] { return feedback.at("velocity")[0].get<double>() > .1; }, "forward feedback");
    sx = 1;
    sy = 0;
    rx = 1;
    wait(
        [&] {
          return last.at("velocity")[1].get<double>() < -.1 &&
                 last.at("velocity")[2].get<double>() < -.1;
        },
        "sideways and yaw mapping");
    grip = 0;
    std::this_thread::sleep_for(150ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      check(last.at("velocity") == luxi::Json::array({0., 0., 0.}), "deadman immediate zero");
    }
    sx = 0;
    rx = 0;
    sy = 1;
    grip = 1;
    wait([&] { return last.at("velocity")[0].get<double>() > .05; }, "resume held grip");
    sending = false;
    std::this_thread::sleep_for(400ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      check(!last.at("enabled") && last.at("velocity") == luxi::Json::array({0., 0., 0.}),
            "Quest timeout zero and disarm");
    }
    sending = true;
    std::this_thread::sleep_for(300ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      check(!last.at("enabled"), "reconnect must not rearm");
    }
    check(!request("start"), "non-neutral enable must reject");
    sy = 0;
    grip = 0;
    std::this_thread::sleep_for(100ms);
    check(request("start"), "explicit rearm");
    sy = 1;
    grip = 1;
    std_msgs::msg::String m;
    m.data = "{\"failure_menu\":true}";
    menu->publish(m);
    wait([&] { return !last.at("enabled").get<bool>(); }, "recording menu must disarm");
    check(!request("start"), "menu enable must reject");
    check(request("stop"), "explicit stop");
    std::cout << "PASS: Quest UDP -> R680 mock feedback, neutral enable, deadman, PS2 mapping, "
                 "timeout/reconnect, menu interlock\n";
    result = 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
  }
  running = false;
  sender.join();
  close(fd);
  rclcpp::shutdown();
  spin.join();
  return result;
}
