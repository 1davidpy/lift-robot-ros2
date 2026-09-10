#include <Eigen/Core>
#include <httplib.h>
#include <luxi/core.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>

void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    httplib::Client http("127.0.0.1", 18020);
    http.set_read_timeout(3);
    for (int i = 0; i < 100 && !http.Get("/api/state"); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto page = http.Get("/");
    check(page && page->status == 200 && page->body.find("机器人控制台") != std::string::npos, "dashboard unavailable");
    check(page->has_header("Content-Security-Policy"), "missing CSP");
    for (auto path : {"/app.js", "/style.css"}) {
      auto asset = http.Get(path); check(asset && asset->status == 200, "asset missing");
    }
    const httplib::Headers token{{"Authorization", "Bearer web-test-token-123456"}};
    auto denied = http.Post("/api/base", "{\"command\":\"start\"}", "application/json");
    check(denied && denied->status == 403, "unauthorized command allowed");
    auto malformed = http.Post("/api/base", token, "{\"command\":\"start\",\"detail\":42}", "application/json");
    check(malformed && malformed->status == 400, "invalid detail not rejected");
    auto missing = http.Post("/api/base", token, "{\"command\":\"start\"}", "application/json");
    check(missing && missing->status == 503, "missing service not reported");
    auto s = luxi::Json::parse(http.Get("/api/state")->body);
    check(!s["system"]["control_authorized"].get<bool>() && !s["quest"]["fresh"].get<bool>(), "initial status wrong");
    s = luxi::Json::parse(http.Get("/api/state", token)->body);
    check(s["system"]["control_authorized"].get<bool>(), "token rejected");
    auto node = std::make_shared<rclcpp::Node>("web_test_publisher");
    auto quest = node->create_publisher<luxi::Quest>("/luxi/quest", 10);
    auto arm = node->create_publisher<luxi::State>("/luxi/arms/state", 1);
    auto base = node->create_publisher<std_msgs::msg::String>("/luxi/base/state", 1);
    auto camera = node->create_publisher<luxi::Frame>("/luxi/cameras/head_rgb", 1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (int i = 0; i < 25; ++i) {
      luxi::Quest q; q.receive_mono_ns = luxi::mono(); quest->publish(q);
      luxi::State a; a.receive_mono_ns = luxi::mono(); a.valid.fill(true); arm->publish(a);
      std_msgs::msg::String b;
      b.data = luxi::Json{{"receive_mono_ns", std::to_string(luxi::mono())}, {"valid", true}}.dump(); base->publish(b);
      luxi::Frame f; f.camera_id = "head_rgb"; f.receive_mono_ns = luxi::mono(); f.sequence = i;
      f.data = {0xff, 0xd8, 0xff, 0xd9}; camera->publish(f);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    s = luxi::Json::parse(http.Get("/api/state")->body);
    check(s["quest"]["fresh"] && s["arms"]["fresh"] && s["base"]["valid"] && s["cameras"]["head_rgb"]["fresh"], "live feedback missing");
    auto img = http.Get("/api/cameras/head_rgb"); check(img && img->status == 200 && img->has_header("X-Frame-Id"), "camera endpoint failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    s = luxi::Json::parse(http.Get("/api/state")->body);
    check(!s["quest"]["fresh"].get<bool>() && !s["arms"]["fresh"].get<bool>() && !s["base"]["valid"].get<bool>() && !s["cameras"]["head_rgb"]["fresh"].get<bool>(), "stale device shown online");
    img = http.Get("/api/cameras/head_rgb"); check(img && img->status == 503, "stale camera exposed");
    std::cout << "Web assets, auth, validation, unavailable services, live/stale devices and camera expiry passed\n";
    rclcpp::shutdown(); return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
}
