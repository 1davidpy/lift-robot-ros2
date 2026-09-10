// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.



// clang-format off
// Eigen must precede the _res resolver macro exposed by httplib on Ubuntu 22.04.
#include <Eigen/Core>
#include <httplib.h>
// clang-format on
#include <luxi/storage.hpp>
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <thread>
class Server : public rclcpp::Node {
  httplib::Server http_;
  std::thread thread_;
  std::mutex mutex_;
  luxi::fs::path root_;
  std::string token_;
  luxi::Json state_, control_, recorder_, base_, base_status_;
  std::map<std::string, luxi::Frame> frames_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Client<luxi::Service>::SharedPtr arm_client_, record_client_, base_client_;

 public:
  Server() : Node("data_server") {
    root_ = declare_parameter<std::string>("directory", "data");
    luxi::fs::create_directories(root_);
    root_ = luxi::fs::canonical(root_);
    auto host = declare_parameter("host", "127.0.0.1");
    int port = declare_parameter("port", 8000);
    token_ = declare_parameter<std::string>("control_token", "");
    if (host != "127.0.0.1" && token_.empty())
      RCLCPP_WARN(get_logger(), "Remote server is read-only until control_token is configured");
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
        std::lock_guard<std::mutex> lock(mutex_); base_status_ = j;
      }));
    subs_.push_back(create_subscription<luxi::State>(
        "/luxi/arms/state", 1, [this](luxi::State::ConstSharedPtr s) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = {{"schema", "luxi_arms_v1"},
                    {"position", s->position},
                    {"velocity", s->velocity},
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
            if (std::string(topic) == "/luxi/control_status")
              control_ = j;
            else
              recorder_ = j;
          }));
    for (auto camera : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"})
      subs_.push_back(create_subscription<luxi::Frame>(std::string("/luxi/cameras/") + camera, 1,
                                                       [this](luxi::Frame::ConstSharedPtr f) {
                                                         std::lock_guard<std::mutex> lock(mutex_);
                                                         frames_[f->camera_id] = *f;
                                                       }));
    http_.Get("/api/state", [this](const auto&, auto& res) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto base = base_;
      if (base.is_object() && base.contains("receive_mono_ns"))
        base["valid"] = luxi::fresh(std::stoll(base.at("receive_mono_ns").get<std::string>()), luxi::mono(), 250000000);
      res.set_content(luxi::Json{{"arms", state_}, {"base", base}, {"base_status", base_status_},
                                 {"control", control_},
                                 {"recorder", recorder_},
                                 {"server_mono_ns", std::to_string(luxi::mono())}}
                          .dump(),
                      "application/json");
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
      res.set_content(luxi::config(p).dump(), "application/json");
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
    http_.Post("/api/(control|episode|base)", [this](const auto& req, auto& res) {
      if ((token_.empty() && req.remote_addr != "127.0.0.1") ||
          (!token_.empty() && req.get_header_value("Authorization") != "Bearer " + token_)) {
        res.status = 403;
        return;
      }
      auto j = luxi::Json::parse(req.body, nullptr, false);
      if (j.is_discarded() || !j.is_object() || !j.contains("command") ||
          !j["command"].is_string()) {
        res.status = 400;
        return;
      }
      auto client = req.matches[1] == "control" ? arm_client_ : req.matches[1] == "base" ? base_client_ : record_client_;
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
    http_.Get("/", [](const auto&, auto& res) {
      res.set_content(
          R"html(<!doctype html><meta charset="utf-8"><title>LUXI 数据采集</title><style>body{font:16px sans-serif;margin:30px;background:#eef3f8;color:#123}button{padding:12px;margin:5px}img{width:31%;margin:1%}pre{white-space:pre-wrap}</style><h1>LUXI 双臂遥操作与数据采集</h1><p>先使能并等待对齐，再开始录制。暂停会按配置返回起始姿态。</p><input id="task" value="teleoperation" placeholder="任务说明"><input id="token" type="password" placeholder="远程控制令牌"><div><button onclick="cmd('control','start')">使能双臂</button><button onclick="cmd('control','stop')">暂停 / 返回</button><button onclick="cmd('episode','start')">开始录制</button><button onclick="cmd('episode','success')">成功并保存</button><button onclick="cmd('episode','fail')">失败并保存</button><button onclick="cmd('episode','cancel')">取消并保留</button></div><div><button onclick="cmd('base','start')">启用底盘（摇杆回中，松开左握持键）</button><button onclick="cmd('base','stop')">底盘停车</button><span>启用后按住左握持键行驶；左摇杆平移、右摇杆左右转向。</span></div><p id="result"></p><img id="head_rgb"><img id="left_wrist_rgb"><img id="right_wrist_rgb"><pre id="status"></pre><a href="/api/episodes">已提交 episode 列表</a><script>async function cmd(kind,command){try{let r=await fetch('/api/'+kind,{method:'POST',headers:{'Content-Type':'application/json','Authorization':'Bearer '+document.getElementById('token').value},body:JSON.stringify({command,detail:document.getElementById('task').value})});document.getElementById('result').textContent=await r.text()}catch(e){document.getElementById('result').textContent=e}}setInterval(async()=>{try{document.getElementById('status').textContent=JSON.stringify(await(await fetch('/api/state')).json(),null,2);for(let c of ['head_rgb','left_wrist_rgb','right_wrist_rgb'])document.getElementById(c).src='/api/cameras/'+c+'?t='+Date.now()}catch(e){}},500)</script>)html",
          "text/html");
    });
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
