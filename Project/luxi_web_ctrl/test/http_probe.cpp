#include <Eigen/Core>
#include <httplib.h>
#include <luxi/core.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cstring>
#include <limits>

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
    check(page->body.find("type=\"password\"") == std::string::npos, "password input still present");
    auto malformed = http.Post("/api/base", "{\"command\":\"start\",\"detail\":42}", "application/json");
    check(malformed && malformed->status == 400, "invalid detail not rejected");
    for (auto path : {"/api/base", "/api/control", "/api/episode"}) {
      auto missing = http.Post(path, "{\"command\":\"start\"}", "application/json");
      check(missing && missing->status == 503, "anonymous command did not reach service readiness check");
    }
    auto invalid_device = http.Post("/api/devices/unknown", "{\"action\":\"start\"}", "application/json");
    check(invalid_device && invalid_device->status == 400, "anonymous device request did not reach validation");
    auto s = luxi::Json::parse(http.Get("/api/state")->body);
    check(s["system"]["control_authorized"].get<bool>() && !s["quest"]["fresh"].get<bool>(), "initial password-free status wrong");
    auto node = std::make_shared<rclcpp::Node>("web_test_publisher");
    auto quest = node->create_publisher<luxi::Quest>("/luxi/quest", 10);
    auto arm = node->create_publisher<luxi::State>("/luxi/arms/state", 1);
    auto base = node->create_publisher<std_msgs::msg::String>("/luxi/base/state", 1);
    auto camera = node->create_publisher<luxi::Frame>("/luxi/cameras/head_rgb", 1);
    auto depth = node->create_publisher<luxi::Frame>("/luxi/cameras/head_rgb/depth", rclcpp::SensorDataQoS());
    auto depth_info = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/head_rgb/depth/camera_info", rclcpp::SensorDataQoS());
    auto unavailable = http.Get("/api/cameras/head_rgb/depth");
    check(unavailable && unavailable->status == 503, "missing depth must be unavailable");
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
    luxi::Frame d;
    d.camera_id = "head_rgb"; d.sequence = 42; d.width = 2; d.height = 2;
    d.encoding = "32FC1"; d.timestamp_quality = "mock_host";
    d.header.stamp = node->now(); d.header.frame_id = "head_rgb_depth_optical";
    float pixels[] = {1.25f, 2.f, std::numeric_limits<float>::quiet_NaN(), 4.f};
    d.data.resize(sizeof(pixels)); std::memcpy(d.data.data(), pixels, sizeof(pixels));
    sensor_msgs::msg::CameraInfo calibration;
    calibration.header = d.header; calibration.width = 2; calibration.height = 2;
    calibration.distortion_model = "plumb_bob"; calibration.d.assign(5, 0.);
    calibration.k = {500., 0., 1., 0., 500., 1., 0., 0., 1.};
    for (int i = 0; i < 10; ++i) {
      d.receive_mono_ns = luxi::mono(); depth_info->publish(calibration); depth->publish(d);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    auto raw = http.Get("/api/cameras/head_rgb/depth");
    check(raw && raw->status == 200 && raw->body.size() == sizeof(pixels) &&
          std::memcmp(raw->body.data(), pixels, sizeof(pixels)) == 0, "depth samples changed in transit");
    check(raw->get_header_value("X-Depth-Unit") == "metre" &&
          raw->get_header_value("X-Byte-Order") == "little-endian" &&
          raw->get_header_value("X-Mock") == "true", "depth wire metadata missing");
    check(raw->get_header_value("X-Optical-Frame") == d.header.frame_id &&
          raw->has_header("X-Stamp-Sec") && raw->has_header("X-Stamp-Nanosec") &&
          raw->has_header("X-Age-Ms"), "depth frame/timestamp headers missing");
    auto details = http.Get("/api/cameras/head_rgb/depth/info");
    check(details && details->status == 200, "depth info unavailable");
    auto metadata = luxi::Json::parse(details->body);
    check(metadata["calibration_available"] && metadata["intrinsics"]["K"][0] == 500. &&
          !metadata["aligned_to_color"].get<bool>(), "depth calibration/frame mismatch");
    auto point = http.Get("/api/cameras/head_rgb/depth/point?x=0&y=0");
    check(point && point->status == 200 && luxi::Json::parse(point->body)["depth_m"] == 1.25,
          "pixel depth incorrect");
    point = http.Get("/api/cameras/head_rgb/depth/point?x=0&y=1");
    auto invalid_point = luxi::Json::parse(point->body);
    check(!invalid_point["valid"].get<bool>() && invalid_point["depth_m"].is_null(), "invalid depth was exposed as a distance");
    for (auto query : {"x=2&y=0", "x=-1&y=0", "x=1.5&y=0", "x=999999999999999&y=0", "x=0", "x=0&x=1&y=0"}) {
      auto result = http.Get((std::string("/api/cameras/head_rgb/depth/point?") + query).c_str());
      check(result && result->status == 400, "invalid depth coordinate accepted");
    }
    auto other = http.Get("/api/cameras/left_wrist_rgb/depth");
    check(other && other->status == 503, "camera depth streams mixed");
    // New frame without matching calibration must not inherit an old stamp's K.
    d.header.stamp = node->now();
    for (int i = 0; i < 5; ++i) {
      d.receive_mono_ns = luxi::mono(); depth->publish(d);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    metadata = luxi::Json::parse(http.Get("/api/cameras/head_rgb/depth/info")->body);
    check(!metadata["calibration_available"].get<bool>(), "stale calibration joined to a new frame");
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    s = luxi::Json::parse(http.Get("/api/state")->body);
    check(!s["quest"]["fresh"].get<bool>() && !s["arms"]["fresh"].get<bool>() && !s["base"]["valid"].get<bool>() && !s["cameras"]["head_rgb"]["fresh"].get<bool>(), "stale device shown online");
    img = http.Get("/api/cameras/head_rgb"); check(img && img->status == 503, "stale camera exposed");
    for (auto suffix : {"", "/info", "/point?x=0&y=0"}) {
      auto result = http.Get((std::string("/api/cameras/head_rgb/depth") + suffix).c_str());
      check(result && result->status == 503, "stale depth exposed");
    }
    check(!s["cameras"]["head_rgb"]["depth"]["fresh"].get<bool>(), "stale depth status shown fresh");
    // Recovery, then truncated input: the malformed packet must be ignored.
    d.sequence = 43; d.receive_mono_ns = luxi::mono(); depth->publish(d);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    d.sequence = 44; d.data.pop_back(); depth->publish(d);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    raw = http.Get("/api/cameras/head_rgb/depth");
    check(raw && raw->status == 200 && raw->get_header_value("X-Frame-Id") == "43",
          "depth recovery or malformed-frame rejection failed");
    std::cout << "Password-free web assets/control, validation, unavailable services, live/stale devices and camera expiry passed\n";
    rclcpp::shutdown(); return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
}
