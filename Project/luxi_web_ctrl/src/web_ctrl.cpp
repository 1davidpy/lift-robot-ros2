// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.



// clang-format off
// Eigen must precede the _res resolver macro exposed by httplib on Ubuntu 22.04.
#include <Eigen/Core>
#include <httplib.h>
// clang-format on
#include <luxi/storage.hpp>
#include <luxi/core.hpp>
#include <luxi/receiver_queue.hpp>
#include <mutex>
#include <deque>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <set>
#include <cstdio>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cstring>
#include <luxi/rgbd.hpp>
class Server : public rclcpp::Node {
  httplib::Server http_;
  std::thread thread_;
  std::mutex mutex_;
  luxi::fs::path root_;
  std::unique_ptr<luxi::ReceiverQueue> receiver_;
  bool mock_, base_enabled_, no_cameras_;
  luxi::Quest quest_;
  std::deque<int64_t> quest_arrivals_;
  luxi::Json startup_;
  int64_t startup_time_ = 0;
  rclcpp::Client<luxi::Service>::SharedPtr startup_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr startup_sub_;
  int64_t control_time_ = 0, recorder_time_ = 0, base_status_time_ = 0;
  luxi::Json state_, control_, recorder_, base_, base_status_, lift_, lift_status_;
  int64_t lift_status_time_ = 0;
  std::map<std::string, luxi::Frame> frames_;
  std::map<std::string, luxi::Frame::ConstSharedPtr> depths_;
  std::map<std::string, luxi_interfaces::msg::RGBDFrame::ConstSharedPtr> rgbd_;
  std::map<std::string, std::deque<sensor_msgs::msg::CameraInfo>> depth_info_;
  std::map<std::string, luxi::Json> depth_status_;
  std::map<std::string, luxi::Json> camera_status_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Client<luxi::Service>::SharedPtr arm_client_, record_client_, base_client_, lift_client_;

  rclcpp::Client<luxi::Service>::SharedPtr edge_client_;
  bool edge_locked_ = false;

  // Called with mutex_ held. Reopening only the idle UART service is independent
  // of an already stopped ACT session retaining OpenArm torque.
  bool lift_reconnect_during_arm_hold(int64_t now) const {
    constexpr int64_t status_ttl = 500000000;
    if (!state_.is_object() || !state_.value("enabled", false) ||
        state_.value("phase", std::string()) != "manual_pose_hold" ||
        !luxi::fresh(std::stoll(state_.at("receive_mono_ns").get<std::string>()), now, status_ttl) ||
        !startup_.is_object() || !luxi::fresh(startup_time_, now, status_ttl) ||
        !control_.is_object() || !luxi::fresh(control_time_, now, status_ttl) ||
        control_.value("system_phase", std::string()) != "disabled" ||
        !lift_status_.is_object() || !luxi::fresh(lift_status_time_, now, status_ttl))
      return false;
    const auto act = startup_.value("act", luxi::Json::object());
    const auto stage = act.is_object() ? act.value("stage", std::string()) : std::string();
    return (stage == "stopped" || stage == "fault") &&
           lift_status_.value("transport", std::string()) == "uart" &&
           lift_status_.value("phase", std::string()) == "disabled" &&
           !lift_status_.value("enabled", true) && !lift_status_.value("session", true) &&
           !lift_status_.value("session_present", true) &&
           !lift_status_.value("external_control", true) &&
           !lift_status_.value("dashboard_position", true) &&
           !(lift_status_.value("flags", 255) & (4 | 8 | 64));
  }

 public:
  Server() : Node("web_ctrl") {
    root_ = declare_parameter<std::string>("directory", "data");
    luxi::fs::create_directories(root_);
    root_ = luxi::fs::canonical(root_);
    const bool ram_staging = declare_parameter("ram_staging", false);
    const bool read_only = declare_parameter("read_only", false);
    const bool allow_device_reconnect = declare_parameter("allow_device_reconnect", false);
    if (read_only) {
      http_.set_pre_routing_handler([allow_device_reconnect](const auto& req, auto& res) {
        if (req.method == "GET" || req.method == "HEAD")
          return httplib::Server::HandlerResponse::Unhandled;
        if (allow_device_reconnect && req.method == "POST" && req.path.rfind("/api/devices/", 0) == 0)
          return httplib::Server::HandlerResponse::Unhandled;
        res.status = 405;
        res.set_header("Allow", "GET, HEAD");
        return httplib::Server::HandlerResponse::Handled;
      });
    }
    receiver_ = std::make_unique<luxi::ReceiverQueue>(root_, ram_staging);
    http_.Get("/api/receiver", [this](const auto&, auto& res) {
      res.set_content(receiver_->status().dump(), "application/json");
    });
    http_.Post("/api/receiver", [this](const auto& req, auto& res) {
      if (req.has_header("Origin") || req.get_header_value("Content-Type").find("application/json") != 0) {
        res.status = 403; return;
      }
      try {
        res.set_content(receiver_->request(luxi::Json::parse(req.body), req.remote_addr).dump(), "application/json");
      } catch (const std::exception& e) {
        res.status = 409;
        res.set_content(luxi::Json{{"success", false}, {"message", e.what()}}.dump(), "application/json");
      }
    });
    auto host = declare_parameter("host", "127.0.0.1");
    int port = declare_parameter("port", 8000);
    mock_ = declare_parameter("mock", true);
    base_enabled_ = declare_parameter("base_enabled", false);
    no_cameras_ = declare_parameter("no_cameras", false);
    subs_.push_back(create_subscription<luxi::Quest>("/luxi/quest", 10,
      [this](luxi::Quest::ConstSharedPtr q) {
        std::lock_guard<std::mutex> lock(mutex_);
        quest_ = *q;
        const auto now = luxi::mono();
        quest_arrivals_.push_back(now);
        while (!quest_arrivals_.empty() && now - quest_arrivals_.front() > 1000000000)
          quest_arrivals_.pop_front();
      }));
    startup_client_ = create_client<luxi::Service>("/luxi/arms/startup");
    startup_sub_ = create_subscription<std_msgs::msg::String>("/luxi/arms/startup_status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (j.is_object()) {
            std::lock_guard<std::mutex> lock(mutex_);
            startup_ = j; startup_time_ = luxi::mono();
          }
        });
    edge_client_ = create_client<luxi::Service>("/luxi/edge_lock");
    subs_.push_back(create_subscription<std_msgs::msg::Bool>(
        "/luxi/edge_lock/status", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          edge_locked_ = msg->data;
        }));
    arm_client_ = create_client<luxi::Service>("/luxi/control");
    record_client_ = create_client<luxi::Service>("/luxi/episode");
    base_client_ = create_client<luxi::Service>("/luxi/base/control");
    lift_client_ = create_client<luxi::Service>("/luxi/lift/control");
    for (auto topic : {"/luxi/lift/state", "/luxi/lift/status"})
      subs_.push_back(create_subscription<std_msgs::msg::String>(topic, rclcpp::QoS(10),
        [this, topic](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (!j.is_object()) return;
          std::lock_guard<std::mutex> lock(mutex_);
          if (std::string(topic) == "/luxi/lift/state") lift_ = j;
          else { lift_status_ = j; lift_status_time_ = luxi::mono(); }
        }));
    subs_.push_back(create_subscription<std_msgs::msg::String>("/luxi/base/state", rclcpp::SensorDataQoS().keep_last(1),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        auto j = luxi::Json::parse(m->data, nullptr, false);
        if (j.is_discarded()) return;
        std::lock_guard<std::mutex> lock(mutex_); base_ = j;
      }));
    subs_.push_back(create_subscription<std_msgs::msg::String>("/luxi/base/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        auto j = luxi::Json::parse(m->data, nullptr, false);
        if (j.is_discarded()) return;
        std::lock_guard<std::mutex> lock(mutex_); base_status_ = j; base_status_time_ = luxi::mono();
      }));
    subs_.push_back(create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = {{"schema", "luxi_arms_v1"},
                    {"position", s->position},
                    {"velocity", s->velocity},
                    {"phase", s->phase},
                    {"joint_order", "right_then_left"},
                    {"mos_temperature", s->mos_temperature},
                    {"rotor_temperature", s->rotor_temperature},
                    {"effort", s->effort},
                    {"valid", s->valid},
                    {"age_ns", s->age_ns},
                    {"receive_mono_ns", std::to_string(s->receive_mono_ns)},
                    {"enabled", s->enabled}};
        }));
    for (auto topic : {"/luxi/control_status", "/luxi/recorder/status"})
      subs_.push_back(create_subscription<std_msgs::msg::String>(
          topic, rclcpp::QoS(1).transient_local(),
          [this, topic](std_msgs::msg::String::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto j = luxi::Json::parse(m->data, nullptr, false);
            if (j.is_discarded()) return;
            if (std::string(topic) == "/luxi/control_status") {
              control_ = j; control_time_ = luxi::mono();
            } else {
              recorder_ = j; recorder_time_ = luxi::mono();
            }
          }));
    for (auto camera : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"})
      subs_.push_back(create_subscription<luxi::Frame>(std::string("/luxi/cameras/") + camera, 1,
                                                       [this](luxi::Frame::ConstSharedPtr f) {
                                                         std::lock_guard<std::mutex> lock(mutex_);
                                                         frames_[f->camera_id] = *f;
                                                       }));
    for (auto camera : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"})
      subs_.push_back(create_subscription<std_msgs::msg::String>(std::string("/luxi/cameras/") + camera + "/status",
        rclcpp::QoS(1).transient_local(), [this, camera](std_msgs::msg::String::ConstSharedPtr m) {
          auto j = luxi::Json::parse(m->data, nullptr, false);
          if (!j.is_object() || !j.value("reason", luxi::Json()).is_string() ||
              !j.value("phase", luxi::Json()).is_string()) return;
          std::lock_guard<std::mutex> lock(mutex_);
          j["receive_mono_ns"] = std::to_string(luxi::mono());
          camera_status_[camera] = j;
        }));
    for (auto camera : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"}) {
      const std::string id = camera;
      subs_.push_back(create_subscription<luxi_interfaces::msg::RGBDFrame>("/luxi/cameras/" + id + "/rgbd",
          rclcpp::SensorDataQoS().keep_last(1), [this, id](luxi_interfaces::msg::RGBDFrame::ConstSharedPtr p) {
            if (!luxi::valid_rgbd(*p, id) || !luxi::fresh(p->depth.receive_mono_ns, luxi::mono(), 1000000000)) return;
            std::lock_guard<std::mutex> lock(mutex_); rgbd_[id] = p;
          }));
      subs_.push_back(create_subscription<std_msgs::msg::String>("/luxi/cameras/" + id + "/depth/status",
          rclcpp::QoS(1).transient_local(), [this, id](std_msgs::msg::String::ConstSharedPtr m) {
            auto value = luxi::Json::parse(m->data, nullptr, false);
            if (!value.is_object()) return;
            std::lock_guard<std::mutex> lock(mutex_); depth_status_[id] = value;
          }));
      subs_.push_back(create_subscription<luxi::Frame>("/luxi/cameras/" + id + "/depth",
          rclcpp::SensorDataQoS().keep_last(1), [this, id](luxi::Frame::ConstSharedPtr f) {
            // Reject malformed frames before exposing dimensions or binary samples.
            if (f->camera_id != id || f->encoding != "32FC1" || !f->width || !f->height ||
                f->width > 1920 || f->height > 1080 ||
                f->data.size() != size_t(f->width) * f->height * 4) return;
            std::lock_guard<std::mutex> lock(mutex_); depths_[id] = f;
          }));
      subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
          "/camera/" + id + "/depth/camera_info", rclcpp::SensorDataQoS(),
          [this, id](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto& queue = depth_info_[id]; queue.push_back(*info);
            while (queue.size() > 8) queue.pop_front();
          }));
    }
    http_.Get("/api/cameras/(head_rgb|left_wrist_rgb|right_wrist_rgb)/rgbd(/info)?",
        [this](const auto& req, auto& res) {
          const std::string id = req.matches[1], operation = req.matches[2];
          luxi_interfaces::msg::RGBDFrame::ConstSharedPtr pair;
          { std::lock_guard<std::mutex> lock(mutex_);
            auto it = rgbd_.find(id); if (it != rgbd_.end()) pair = it->second; }
          res.set_header("Cache-Control", "no-store");
          if (!pair || !luxi::fresh(pair->depth.receive_mono_ns, luxi::mono(), 1000000000)) {
            res.status = 503;
            res.set_content(luxi::Json{{"error", "aligned_rgbd_unavailable_or_stale"}, {"camera_id", id}}.dump(), "application/json");
            return;
          }
          res.set_header("X-Pair-Id", pair->pair_id);
          res.set_header("X-Aligned-To-Color", "true");
          auto metadata = luxi::rgbd_metadata(*pair).dump();
          if (operation == "/info") { res.set_content(metadata, "application/json"); return; }
          // One immutable ROS message supplies every part of this response.
          std::string boundary = "luxi_rgbd_" + pair->pair_id;
          std::replace(boundary.begin(), boundary.end(), ':', '_');
          std::string body;
          body.reserve(metadata.size() + pair->color.data.size() + pair->depth.data.size() + 512);
          const auto part = [&](const char* name, const char* type, const char* data, size_t length) {
            body += "--" + boundary + "\r\nContent-Type: " + type +
                "\r\nContent-Disposition: inline; name=\"" + name + "\"\r\n\r\n";
            body.append(data, length); body += "\r\n";
          };
          part("metadata", "application/json", metadata.data(), metadata.size());
          part("color", "image/jpeg", reinterpret_cast<const char*>(pair->color.data.data()), pair->color.data.size());
          part("depth", "application/octet-stream", reinterpret_cast<const char*>(pair->depth.data.data()), pair->depth.data.size());
          body += "--" + boundary + "--\r\n";
          const auto content_type = "multipart/mixed; boundary=" + boundary;
          res.set_content(body, content_type.c_str());
        });
    http_.Get("/api/cameras/(head_rgb|left_wrist_rgb|right_wrist_rgb)/depth(/info|/point)?",
        [this](const auto& req, auto& res) {
          const std::string id = req.matches[1], operation = req.matches[2];
          luxi::Frame::ConstSharedPtr f;
          sensor_msgs::msg::CameraInfo info; bool calibrated = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = depths_.find(id);
            if (it != depths_.end()) f = it->second;
            if (f) for (const auto& candidate : depth_info_[id]) {
              if (candidate.header.stamp == f->header.stamp &&
                  candidate.header.frame_id == f->header.frame_id &&
                  candidate.width == f->width && candidate.height == f->height &&
                  std::isfinite(candidate.k[0]) && candidate.k[0] > 0 &&
                  std::isfinite(candidate.k[4]) && candidate.k[4] > 0) {
                info = candidate; calibrated = true; break;
              }
            }
          }
          const auto now = luxi::mono();
          res.set_header("Cache-Control", "no-store");
          if (!f || !luxi::fresh(f->receive_mono_ns, now, 1000000000)) {
            res.status = 503;
            res.set_content(luxi::Json{{"error", "depth_unavailable_or_stale"}, {"camera_id", id}}.dump(), "application/json");
            return;
          }
          res.set_header("X-Frame-Id", std::to_string(f->sequence));
          res.set_header("X-Receive-Mono-Ns", std::to_string(f->receive_mono_ns));
          res.set_header("X-Timestamp-Quality", f->timestamp_quality);
          res.set_header("X-Depth-Unit", "metre");
          res.set_header("X-Depth-Encoding", "32FC1");
          res.set_header("X-Width", std::to_string(f->width));
          res.set_header("X-Height", std::to_string(f->height));
          res.set_header("X-Optical-Frame", f->header.frame_id);
          res.set_header("X-Stamp-Sec", std::to_string(f->header.stamp.sec));
          res.set_header("X-Stamp-Nanosec", std::to_string(f->header.stamp.nanosec));
          res.set_header("X-Device-Timestamp-Ns", std::to_string(f->device_timestamp));
          res.set_header("X-Age-Ms", std::to_string((now - f->receive_mono_ns) / 1000000));
          res.set_header("X-Byte-Order", "little-endian");
          res.set_header("X-Mock", f->timestamp_quality == "mock_host" ? "true" : "false");
          const bool aligned = f->header.frame_id == id + "_color_optical";
          res.set_header("X-Aligned-To-Color", aligned ? "true" : "false");
          auto metadata = luxi::Json{{"camera_id", id}, {"sequence", f->sequence},
              {"frame_id", f->header.frame_id}, {"width", f->width}, {"height", f->height},
              {"encoding", "32FC1"}, {"unit", "metre"}, {"byte_order", "little-endian"},
              {"step", f->width * 4}, {"invalid_value", "NaN"},
              {"mock", f->timestamp_quality == "mock_host"}, {"aligned_to_color", aligned},
              {"age_ms", (now - f->receive_mono_ns) / 1000000},
              {"receive_mono_ns", std::to_string(f->receive_mono_ns)},
              {"device_timestamp_ns", std::to_string(f->device_timestamp)},
              {"timestamp_quality", f->timestamp_quality},
              {"stamp", {{"sec", f->header.stamp.sec}, {"nanosec", f->header.stamp.nanosec}}},
              {"calibration_available", calibrated}};
          if (calibrated) metadata["intrinsics"] = {{"distortion_model", info.distortion_model},
              {"K", info.k}, {"D", info.d}, {"R", info.r}, {"P", info.p}};
          if (operation == "/info") {
            res.set_content(metadata.dump(), "application/json"); return;
          }
          if (operation == "/point") {
            auto coordinate = [&req](const char* name, uint32_t limit) -> int {
              if (req.get_param_value_count(name) != 1) return -1;
              auto text = req.get_param_value(name);
              if (text.empty() || text.size() > 5 || text.find_first_not_of("0123456789") != std::string::npos) return -1;
              const auto value = std::stoul(text);
              return value < limit ? static_cast<int>(value) : -1;
            };
            int x = coordinate("x", f->width), y = coordinate("y", f->height);
            if (x < 0 || y < 0) {
              res.status = 400;
              res.set_content("{\"error\":\"x_y_must_be_in_range_integer_depth_pixels\"}", "application/json"); return;
            }
            float metres;
            std::memcpy(&metres, f->data.data() + (size_t(y) * f->width + x) * 4, 4);
            bool valid = std::isfinite(metres) && metres > 0;
            metadata["x"] = x; metadata["y"] = y; metadata["valid"] = valid;
            metadata["depth_m"] = valid ? luxi::Json(metres) : luxi::Json(nullptr);
            res.set_content(metadata.dump(), "application/json"); return;
          }
          res.set_content(reinterpret_cast<const char*>(f->data.data()), f->data.size(), "application/octet-stream");
        });
    // Fixed helper and allowlisted arguments only; never execute request text as shell code.
    auto management = [](const std::string& arguments) {
      std::string output;
      FILE* pipe = popen(("/usr/bin/timeout 8 /usr/bin/sudo -n /usr/local/libexec/luxi-service-control " + arguments + " 2>/dev/null").c_str(), "r");
      if (pipe) { char buffer[4096]; while (fgets(buffer, sizeof(buffer), pipe)) output += buffer; pclose(pipe); }
      auto result = luxi::Json::parse(output, nullptr, false);
      if (!result.is_object()) return luxi::Json{{"success", false}, {"message", "设备管理服务未安装或响应超时"}};
      return result;
    };
    http_.Get("/api/devices", [management, read_only, allow_device_reconnect](const auto&, auto& res) {
      auto result = management("status");
      result["management_mode"] = !read_only ? "full" : allow_device_reconnect ? "reconnect" : "read_only";
      FILE* pipe = popen("/usr/sbin/ip -j -details link show type can 2>/dev/null", "r");
      if (pipe) {
        std::string output;
        char buffer[4096];
        while (fgets(buffer, sizeof(buffer), pipe)) output += buffer;
        if (!pclose(pipe)) {
          const auto links = luxi::Json::parse(output, nullptr, false);
          if (links.is_array()) for (const auto& link : links) {
            const auto name = link.value("ifname", std::string());
            if (name != "luxi_left" && name != "luxi_right" && name != "luxi_base") continue;
            const auto info = link.value("linkinfo", luxi::Json::object()).value("info_data", luxi::Json::object());
            result["can"][name] = info.value("state", "UNKNOWN");
          }
        }
      }
      res.status = result.value("success", false) ? 200 : 503;
      res.set_header("Cache-Control", "no-store");
      res.set_content(result.dump(), "application/json");
    });
    http_.Post("/api/devices/([a-z_]+)", [this, management, read_only](const auto& req, auto& res) {
      auto fail = [&](int code, const char* message) { res.status = code; res.set_content(luxi::Json{{"success", false}, {"message", message}}.dump(), "application/json"); };
      const std::string id = req.matches[1];
      const std::set<std::string> ids{"arms", "base", "lift", "quest", "program", "head_rgb", "left_wrist_rgb", "right_wrist_rgb"};
      auto body = luxi::Json::parse(req.body, nullptr, false);
      if (!ids.count(id) || req.get_header_value("Content-Type").find("application/json") != 0 || !body.is_object() || !body.value("action", luxi::Json()).is_string()) { fail(400, "无效设备或操作"); return; }
      auto action = body.at("action").template get<std::string>();
      if (action != "start" && action != "stop" && action != "restart") { fail(400, "无效操作"); return; }
      if (read_only && action != "restart") { fail(403, "此网关仅开放采集重连"); return; }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = luxi::mono();
        if (id == "arms" && state_.is_object() && state_.value("enabled", false)) {
          fail(409, "双臂仍有使能记录，请确认失能后再重连"); return;
        }
        if (id == "arms" && startup_.is_object()) {
          const auto act = startup_.value("act", luxi::Json::object());
          const auto stage = act.value("stage", std::string());
          if (stage == "homing" || stage == "ready" || stage == "running") {
            fail(409, "请先结束 ACT 会话，再重连双臂"); return;
          }
          const auto pose = startup_.value("manual_pose", luxi::Json::object());
          if (pose.is_object()) {
            const auto phase = pose.value("phase", std::string());
            if (phase == "editing" || phase == "preparing" || phase == "holding" || phase == "moving") {
              fail(409, "请先退出双臂动作设置，再管理采集服务"); return;
            }
          }
        }
        const bool arm_interlock = state_.is_object() && state_.value("enabled", false) &&
            luxi::fresh(std::stoll(state_.at("receive_mono_ns").template get<std::string>()), now, 500000000);
        const bool independent_lift_restart = id == "lift" && action == "restart" && lift_reconnect_during_arm_hold(now);
        if ((arm_interlock && !independent_lift_restart) ||
            (base_status_.is_object() && base_status_.value("enabled", false) && luxi::fresh(base_status_time_, now, 500000000)) ||
            (lift_status_.is_object() && lift_status_.value("enabled", false) && luxi::fresh(lift_status_time_, now, 500000000)) ||
            (recorder_.is_object() && recorder_.value("state", std::string()) == "recording" && luxi::fresh(recorder_time_, now, 2000000000))) {
          fail(409, "请先撤销电机使能并结束录制，再管理采集服务"); return;
        }
      }
      auto result = management(action + " " + id);
      res.status = result.value("success", false) ? 202 : 503;
      res.set_content(result.dump(), "application/json");
    });
    http_.Get("/api/state", [this, ram_staging, read_only, allow_device_reconnect](const auto&, auto& res) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = luxi::mono();
      auto base = base_, arms = state_, control = control_, recorder = recorder_, base_status = base_status_;
      auto lift = lift_, lift_status = lift_status_;
      auto startup = startup_;
      if (startup.is_object()) startup["fresh"] = luxi::fresh(startup_time_, now, 500000000);
      const auto recent = [now](const luxi::Json& j, int64_t ttl) {
        return j.is_object() && j.contains("receive_mono_ns") &&
          luxi::fresh(std::stoll(j.at("receive_mono_ns").template get<std::string>()), now, ttl);
      };
      if (base.is_object()) base["valid"] = base.value("valid", false) && recent(base, 250000000);
      if (lift.is_object()) {
        double elapsed_ms = lift.contains("receive_mono_ns") ?
            (now - std::stoll(lift.at("receive_mono_ns").template get<std::string>())) / 1e6 : 1e9;
        lift["valid"] = lift.value("valid", false) && recent(lift, 300000000) &&
            lift.value("effective_age_ms", 1e9) + std::max(0.0, elapsed_ms) <= 250;
      }
      if (lift_status.is_object()) lift_status["fresh"] = luxi::fresh(lift_status_time_, now, 500000000);
      if (arms.is_object()) arms["fresh"] = recent(arms, 150000000);
      if (control.is_object()) {
        control["fresh"] = luxi::fresh(control_time_, now, 500000000);
        control["vr_fresh"] = control.value("vr_fresh", false) && luxi::fresh(quest_.receive_mono_ns, now, 150000000);
      }
      if (recorder.is_object()) recorder["fresh"] = luxi::fresh(recorder_time_, now, 1000000000);
      if (base_status.is_object()) {
        base_status["fresh"] = luxi::fresh(base_status_time_, now, 500000000);
        base_status["feedback_fresh"] = base_status.value("feedback_fresh", false) && recent(base, 100000000);
      }
      size_t rate = 0;
      for (auto t : quest_arrivals_) if (now - t <= 1000000000) ++rate;
      luxi::Json cameras = luxi::Json::object();
      for (auto id : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"}) {
        const auto it = frames_.find(id);
        cameras[id] = {{"fresh", false}, {"enabled", !no_cameras_}};
        cameras[id]["depth"] = {{"fresh", false}};
        auto pair = rgbd_.find(id);
        cameras[id]["rgbd"] = {{"fresh", false}, {"url", std::string("/api/cameras/") + id + "/rgbd"}};
        if (pair != rgbd_.end()) {
          cameras[id]["rgbd"]["fresh"] = luxi::fresh(pair->second->depth.receive_mono_ns, now, 1000000000);
          cameras[id]["rgbd"]["pair_id"] = pair->second->pair_id;
          cameras[id]["rgbd"]["aligned_to_color"] = true;
        }
        auto depth = depths_.find(id);
        if (depth != depths_.end()) {
          const auto& f = *depth->second;
          cameras[id]["depth"] = {{"fresh", luxi::fresh(f.receive_mono_ns, now, 1000000000)},
              {"width", f.width}, {"height", f.height}, {"sequence", f.sequence},
              {"encoding", f.encoding}, {"unit", "metre"}, {"mock", f.timestamp_quality == "mock_host"},
              {"aligned_to_color", f.header.frame_id == std::string(id) + "_color_optical"},
              {"url", std::string("/api/cameras/") + id + "/depth"}};
        }
        auto depth_status = depth_status_.find(id);
        if (depth_status != depth_status_.end()) cameras[id]["depth"]["driver"] = depth_status->second;
        auto status = camera_status_.find(id);
        if (status != camera_status_.end()) {
          cameras[id]["driver"] = status->second;
          cameras[id]["driver"]["fresh"] = recent(status->second, 6000000000);
        }
        if (it != frames_.end()) {
          const auto& f = it->second;
          cameras[id].update({{"fresh", luxi::fresh(f.receive_mono_ns, now, 1000000000)},
            {"width", f.width}, {"height", f.height}, {"sequence", f.sequence},
            {"age_ms", (now - f.receive_mono_ns) / 1000000}, {"timestamp_quality", f.timestamp_quality}});
        }
      }
      std::error_code ec;
      auto disk = luxi::fs::space(root_, ec);
      res.set_content(luxi::Json{
        {"edge_locked", edge_locked_}, {"arm_startup", startup}, {"arms", arms}, {"base", base}, {"base_status", base_status}, {"control", control}, {"recorder", recorder},
        {"lift", lift}, {"lift_status", lift_status},
        {"quest", {{"fresh", luxi::fresh(quest_.receive_mono_ns, now, 200000000)}, {"rate_hz", rate},
          {"receive_mono_ns", std::to_string(quest_.receive_mono_ns)},
          {"left_tracking", quest_.left_tracking}, {"right_tracking", quest_.right_tracking},
          {"sticks", quest_.sticks}, {"grips", quest_.grips}}}, {"cameras", cameras},
        {"system", {{"mock", mock_}, {"base_enabled", base_enabled_}, {"no_cameras", no_cameras_},
          {"data_directory", root_.string()}, {"free_gib", ec ? -1.0 : disk.available / 1073741824.0},
          {"control_authorized", !read_only},
          {"device_management_mode", !read_only ? "full" : allow_device_reconnect ? "reconnect" : "read_only"},
          {"lift_reconnect_during_arm_hold", lift_reconnect_during_arm_hold(now)},
          {"services", {{"edge_lock", edge_client_->service_is_ready()}, {"arm_startup", startup_client_->service_is_ready()}, {"control", arm_client_->service_is_ready()}, {"base", base_client_->service_is_ready()}, {"lift", lift_client_->service_is_ready()}, {"episode", record_client_->service_is_ready()}}}}},
        {"receiver", receiver_->status()}, {"ram_staging", ram_staging},
        {"server_mono_ns", std::to_string(now)}}.dump(), "application/json");
      res.set_header("Cache-Control", "no-store");
    });
    http_.Get("/api/cameras/(head_rgb|left_wrist_rgb|right_wrist_rgb)", [this](const auto& req,
                                                                               auto& res) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = frames_.find(req.matches[1]);
      if (it == frames_.end() ||
          !luxi::fresh(it->second.receive_mono_ns, luxi::mono(), 1000000000)) {
        res.status = 503;
        return;
      }
      auto& f = it->second;
      res.set_header("X-Frame-Id", std::to_string(f.sequence));
      res.set_header("X-Receive-Mono-Ns", std::to_string(f.receive_mono_ns));
      res.set_header("X-Device-Timestamp", std::to_string(f.device_timestamp));
      res.set_header("X-Timestamp-Quality", f.timestamp_quality);
      res.set_header("Cache-Control", "no-store");
      res.set_content(reinterpret_cast<const char*>(f.data.data()), f.data.size(), "image/jpeg");
    });
    http_.Get("/api/episodes", [this](const auto&, auto& res) {
      auto list = luxi::Json::array();
      for (const auto& e : luxi::fs::directory_iterator(root_))
        if (e.is_directory() && !e.is_symlink() && luxi::simple_id(e.path().filename()) &&
            luxi::fs::exists(e.path() / "manifest.json"))
          list.push_back(e.path().filename().string());
      res.set_content(list.dump(), "application/json");
    });
    http_.Get("/api/episodes/([A-Za-z0-9_-]+)/manifest", [this](const auto& req, auto& res) {
      auto p = root_ / req.matches[1].str() / "manifest.json";
      if (!luxi::fs::exists(p) || luxi::fs::is_symlink(p.parent_path()) ||
          luxi::fs::is_symlink(p)) {
        res.status = 404;
        return;
      }
      std::ifstream in(p, std::ios::binary);
      std::ostringstream body; body << in.rdbuf();
      res.set_content(body.str(), "application/json");
    });
    http_.Get("/api/files/([A-Za-z0-9_-]+)/(.+)", [this](const auto& req, auto& res) {
      auto dir = root_ / req.matches[1].str();
      if (!luxi::fs::exists(dir / "manifest.json") || luxi::fs::is_symlink(dir)) {
        res.status = 404;
        return;
      }
      auto name = req.matches[2].str();
      auto manifest = luxi::config(dir / "manifest.json");
      bool allowed = false;
      std::string hash;
      for (const auto& file : manifest.at("files"))
        if (file.at("path") == name) {
          allowed = true;
          hash = file.at("sha256");
        }
      auto path = dir / name;
      if (!allowed || !luxi::fs::exists(path) || luxi::fs::is_symlink(path) ||
          luxi::fs::relative(luxi::fs::canonical(path), luxi::fs::canonical(dir))
                  .string()
                  .rfind("..", 0) == 0) {
        res.status = 404;
        return;
      }
      res.set_header("ETag", "\"" + hash + "\"");
      res.set_header("X-SHA256", hash);
      res.set_header("Accept-Ranges", "bytes");
      res.set_content_provider(luxi::fs::file_size(path), "application/octet-stream",
                               [path](size_t offset, size_t length, httplib::DataSink& sink) {
                                 std::ifstream in(path, std::ios::binary);
                                 in.seekg(offset);
                                 char buf[65536];
                                 while (length) {
                                   size_t size = std::min(length, sizeof(buf));
                                   in.read(buf, size);
                                   auto n = in.gcount();
                                   if (n <= 0) return false;
                                   if (!sink.write(buf, n)) return false;
                                   length -= n;
                                 }
                                 return true;
                               });
    });
    http_.Post("/api/(control|episode|base|keyboard|lift|arm_startup|edge_lock)", [this](const auto& req, auto& res) {
      const bool keyboard = req.matches[1] == "keyboard";
      auto j = luxi::Json::parse(req.body, nullptr, false);
      if (j.is_discarded() || !j.is_object() || !j.contains("command") ||
          !j["command"].is_string() || (j.contains("detail") && !j["detail"].is_string())) {
        res.status = 400;
        return;
      }
      if (keyboard && (req.get_header_value("Content-Type").find("application/json") != 0 ||
          (j["command"] != "keyboard_start" && j["command"] != "keyboard" && j["command"] != "stop"))) {
        res.status = 400;
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string kind = req.matches[1], command = j["command"];
        const auto source = control_.is_object() ? control_.value("control_source", "legacy") : "legacy";
        const bool select = kind == "control" && (command == "select_console" || command == "select_teleop");
        const bool stop = kind == "control" ? (command == "disable" || command == "halt" || command == "hold") :
            (command == "stop" || command == "disable" || command == "joint_stop" || command == "pose_cancel" || command == "position_stop");
        if (source != "legacy" && !select && !stop && kind != "episode" &&
            (!luxi::fresh(control_time_, luxi::mono(), 500000000) ||
             ((kind == "control" || kind == "edge_lock") ? source != "teleop" : source != "console"))) {
          res.status = 409;
          res.set_content(luxi::Json{{"success", false}, {"message", "请先显式切换控制权限"}}.dump(), "application/json");
          return;
        }
      }
      auto client = req.matches[1] == "edge_lock" ? edge_client_ : req.matches[1] == "arm_startup" ? startup_client_ : req.matches[1] == "lift" ? lift_client_ : req.matches[1] == "control" ? arm_client_ : (req.matches[1] == "base" || keyboard) ? base_client_ : record_client_;
      if (!client->service_is_ready()) {
        res.status = 503;
        return;
      }
      auto request = std::make_shared<luxi::Service::Request>();
      request->command = j["command"];
      request->detail = j.value("detail", "");
      auto future = client->async_send_request(request);
      if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        client->remove_pending_request(future);
        res.status = 504;
        return;
      }
      auto result = future.get();
      res.status = result->success ? 200 : 409;
      res.set_content(luxi::Json{{"success", result->success}, {"message", result->message}}.dump(),
                      "application/json");
    });
    const auto web = ament_index_cpp::get_package_share_directory("luxi_web_ctrl") + "/web";
    if (!http_.set_mount_point("/", web)) throw std::runtime_error("Web assets missing");
    http_.set_default_headers({{"X-Content-Type-Options", "nosniff"}, {"Cache-Control", "no-store"},
      {"Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'"}});
    http_.set_payload_max_length(16384);
    http_.set_exception_handler([](const auto&, auto& res, std::exception&) {
      res.status = 500;
      res.set_content("request failed", "text/plain");
    });
    if (!http_.bind_to_port(host.c_str(), port)) throw std::runtime_error("HTTP port busy");
    thread_ = std::thread([this] { http_.listen_after_bind(); });
  }
  ~Server() {
    http_.stop();
    if (thread_.joinable()) thread_.join();
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Server>(); });
}
