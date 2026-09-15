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
#include <thread>
#include <set>
#include <cstdio>
class Server : public rclcpp::Node {
  httplib::Server http_;
  std::thread thread_;
  std::mutex mutex_;
  luxi::fs::path root_;
  std::unique_ptr<luxi::ReceiverQueue> receiver_;
  bool mock_, base_enabled_, no_cameras_;
  luxi::Quest quest_;
  std::deque<int64_t> quest_arrivals_;
  int64_t control_time_ = 0, recorder_time_ = 0, base_status_time_ = 0;
  luxi::Json state_, control_, recorder_, base_, base_status_;
  std::map<std::string, luxi::Frame> frames_;
  std::map<std::string, luxi::Json> camera_status_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Client<luxi::Service>::SharedPtr arm_client_, record_client_, base_client_;

 public:
  Server() : Node("web_ctrl") {
    root_ = declare_parameter<std::string>("directory", "data");
    luxi::fs::create_directories(root_);
    root_ = luxi::fs::canonical(root_);
    const bool ram_staging = declare_parameter("ram_staging", false);
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
    arm_client_ = create_client<luxi::Service>("/luxi/control");
    record_client_ = create_client<luxi::Service>("/luxi/episode");
    base_client_ = create_client<luxi::Service>("/luxi/base/control");
    subs_.push_back(create_subscription<std_msgs::msg::String>("/luxi/base/state", 1,
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
    // Fixed helper and allowlisted arguments only; never execute request text as shell code.
    auto management = [](const std::string& arguments) {
      std::string output;
      FILE* pipe = popen(("/usr/bin/timeout 8 /usr/bin/sudo -n /usr/local/libexec/luxi-service-control " + arguments + " 2>/dev/null").c_str(), "r");
      if (pipe) { char buffer[4096]; while (fgets(buffer, sizeof(buffer), pipe)) output += buffer; pclose(pipe); }
      auto result = luxi::Json::parse(output, nullptr, false);
      if (!result.is_object()) return luxi::Json{{"success", false}, {"message", "设备管理服务未安装或响应超时"}};
      return result;
    };
    http_.Get("/api/devices", [management](const auto&, auto& res) {
      auto result = management("status");
      res.status = result.value("success", false) ? 200 : 503;
      res.set_header("Cache-Control", "no-store");
      res.set_content(result.dump(), "application/json");
    });
    http_.Post("/api/devices/([a-z_]+)", [this, management](const auto& req, auto& res) {
      auto fail = [&](int code, const char* message) { res.status = code; res.set_content(luxi::Json{{"success", false}, {"message", message}}.dump(), "application/json"); };
      const std::string id = req.matches[1];
      const std::set<std::string> ids{"arms", "base", "quest", "program", "head_rgb", "left_wrist_rgb", "right_wrist_rgb"};
      auto body = luxi::Json::parse(req.body, nullptr, false);
      if (!ids.count(id) || req.get_header_value("Content-Type").find("application/json") != 0 || !body.is_object() || !body.value("action", luxi::Json()).is_string()) { fail(400, "无效设备或操作"); return; }
      auto action = body.at("action").template get<std::string>();
      if (action != "start" && action != "stop" && action != "restart") { fail(400, "无效操作"); return; }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if ((state_.is_object() && state_.value("enabled", false) && luxi::fresh(std::stoll(state_.at("receive_mono_ns").template get<std::string>()), luxi::mono(), 500000000)) ||
            (base_status_.is_object() && base_status_.value("enabled", false) && luxi::fresh(base_status_time_, luxi::mono(), 500000000)) ||
            (recorder_.is_object() && recorder_.value("state", std::string()) == "recording" && luxi::fresh(recorder_time_, luxi::mono(), 2000000000))) {
          fail(409, "请先撤销电机使能并结束录制，再管理采集服务"); return;
        }
      }
      auto result = management(action + " " + id);
      res.status = result.value("success", false) ? 202 : 503;
      res.set_content(result.dump(), "application/json");
    });
    http_.Get("/api/state", [this, ram_staging](const auto&, auto& res) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = luxi::mono();
      auto base = base_, arms = state_, control = control_, recorder = recorder_, base_status = base_status_;
      const auto recent = [now](const luxi::Json& j, int64_t ttl) {
        return j.is_object() && j.contains("receive_mono_ns") &&
          luxi::fresh(std::stoll(j.at("receive_mono_ns").template get<std::string>()), now, ttl);
      };
      if (base.is_object()) base["valid"] = base.value("valid", false) && recent(base, 250000000);
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
        {"arms", arms}, {"base", base}, {"base_status", base_status}, {"control", control}, {"recorder", recorder},
        {"quest", {{"fresh", luxi::fresh(quest_.receive_mono_ns, now, 200000000)}, {"rate_hz", rate},
          {"left_tracking", quest_.left_tracking}, {"right_tracking", quest_.right_tracking},
          {"sticks", quest_.sticks}, {"grips", quest_.grips}}}, {"cameras", cameras},
        {"system", {{"mock", mock_}, {"base_enabled", base_enabled_}, {"no_cameras", no_cameras_},
          {"data_directory", root_.string()}, {"free_gib", ec ? -1.0 : disk.available / 1073741824.0},
          {"control_authorized", true},
          {"services", {{"control", arm_client_->service_is_ready()}, {"base", base_client_->service_is_ready()}, {"episode", record_client_->service_is_ready()}}}}},
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
    http_.Post("/api/(control|episode|base|keyboard)", [this](const auto& req, auto& res) {
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
      auto client = req.matches[1] == "control" ? arm_client_ : (req.matches[1] == "base" || keyboard) ? base_client_ : record_client_;
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
