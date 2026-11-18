#include <rclcpp/rclcpp.hpp>
#include <luxi_interfaces/msg/camera_frame.hpp>
#include <luxi_interfaces/msg/arm_state.hpp>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

using Json = nlohmann::json;
using Frame = luxi_interfaces::msg::CameraFrame;
using State = luxi_interfaces::msg::ArmState;
static int64_t mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

class RgbHttp : public rclcpp::Node {
  const std::array<std::string, 3> ids_{"head_rgb", "left_wrist_rgb", "right_wrist_rgb"};
  std::array<Frame::ConstSharedPtr, 3> frames_{};
  State::ConstSharedPtr state_;
  std::vector<rclcpp::Subscription<Frame>::SharedPtr> subscriptions_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  std::mutex mutex_;
  httplib::Server http_;
  std::thread thread_;
  int64_t image_age_ns_, skew_ns_, state_age_ns_;
  bool fresh(int64_t stamp, int64_t now, int64_t bound) const {
    return stamp > 0 && stamp <= now && now - stamp <= bound;
  }
  Json frame_info(const Frame::ConstSharedPtr& f, int64_t now) const {
    if (!f) return {{"available", false}, {"fresh", false}};
    return {{"available", true}, {"fresh", fresh(f->receive_mono_ns, now, image_age_ns_)},
      {"sequence", std::to_string(f->sequence)}, {"width", f->width}, {"height", f->height},
      {"encoding", f->encoding}, {"receive_mono_ns", std::to_string(f->receive_mono_ns)},
      {"device_timestamp", std::to_string(f->device_timestamp)},
      {"timestamp_quality", f->timestamp_quality},
      {"age_ms", (now - f->receive_mono_ns) / 1e6}};
  }
  static void json_response(httplib::Response& res, const Json& j, int status = 200) {
    res.status = status;
    res.set_header("Cache-Control", "no-store");
    res.set_content(j.dump(), "application/json");
  }
 public:
  RgbHttp() : Node("act_rgb_http") {
    const auto host = declare_parameter<std::string>("host", "0.0.0.0");
    const auto port = declare_parameter<int>("port", 18040);
    const auto age = declare_parameter<int>("max_image_age_ms", 200);
    const auto skew = declare_parameter<int>("max_image_skew_ms", 100);
    const auto state_age = declare_parameter<int>("max_state_age_ms", 150);
    if (port < 1024 || port > 65535 || age < 1 || skew < 1 || state_age < 1)
      throw std::runtime_error("Invalid HTTP configuration");
    image_age_ns_ = int64_t(age) * 1000000;
    skew_ns_ = int64_t(skew) * 1000000;
    state_age_ns_ = int64_t(state_age) * 1000000;
    for (size_t i = 0; i < ids_.size(); ++i) {
      subscriptions_.push_back(create_subscription<Frame>("/luxi/cameras/" + ids_[i],
        rclcpp::SensorDataQoS().keep_last(1), [this, i](Frame::ConstSharedPtr f) {
          if (f->camera_id != ids_[i] || (f->encoding != "jpeg" && f->encoding != "jpg") ||
              !f->width || !f->height || f->data.size() < 4 || f->data.size() > 8*1024*1024 ||
              f->data[0] != 0xff || f->data[1] != 0xd8) return;
          std::lock_guard<std::mutex> lock(mutex_); frames_[i] = f;
        }));
    }
    state_sub_ = create_subscription<State>("/luxi/arms/state", rclcpp::SensorDataQoS().keep_last(1),
      [this](State::ConstSharedPtr s) { std::lock_guard<std::mutex> lock(mutex_); state_ = s; });
    http_.new_task_queue = [] { return new httplib::ThreadPool(4); };
    http_.set_read_timeout(2, 0); http_.set_write_timeout(2, 0);
    http_.set_payload_max_length(1024);
    http_.Get("/healthz", [](const auto&, auto& res) {
      json_response(res, {{"service", "act_rgb_http"}, {"ok", true}, {"read_only", true}});
    });
    http_.Get("/v1/cameras", [this](const auto&, auto& res) {
      std::array<Frame::ConstSharedPtr, 3> frames;
      { std::lock_guard<std::mutex> lock(mutex_); frames = frames_; }
      auto now = mono(); Json cameras = Json::object();
      for (size_t i = 0; i < ids_.size(); ++i) cameras[ids_[i]] = frame_info(frames[i], now);
      json_response(res, {{"schema", "luxi_act_rgb_v1"}, {"server_mono_ns", std::to_string(now)},
        {"max_image_age_ms", image_age_ns_/1000000}, {"cameras", cameras}});
    });
    http_.Get("/v1/cameras/(head_rgb|left_wrist_rgb|right_wrist_rgb)/rgb", [this](const auto& req, auto& res) {
      size_t i = 0; while (ids_[i] != req.matches[1].str()) ++i;
      Frame::ConstSharedPtr f;
      { std::lock_guard<std::mutex> lock(mutex_); f = frames_[i]; }
      if (!f || !fresh(f->receive_mono_ns, mono(), image_age_ns_)) {
        json_response(res, {{"error", "image_missing_or_stale"}}, 503); return;
      }
      res.set_header("Cache-Control", "no-store");
      res.set_header("X-Frame-Sequence", std::to_string(f->sequence));
      res.set_header("X-Receive-Mono-Ns", std::to_string(f->receive_mono_ns));
      res.set_header("X-Width", std::to_string(f->width)); res.set_header("X-Height", std::to_string(f->height));
      res.set_content(reinterpret_cast<const char*>(f->data.data()), f->data.size(), "image/jpeg");
    });
    http_.Get("/v1/observation", [this](const auto&, auto& res) {
      std::array<Frame::ConstSharedPtr, 3> frames; State::ConstSharedPtr state;
      { std::lock_guard<std::mutex> lock(mutex_); frames = frames_; state = state_; }
      const auto now = mono(); int64_t oldest = now, newest = 0;
      Json metadata = {{"schema", "luxi_act_observation_v1"}, {"server_mono_ns", std::to_string(now)},
        {"synchronization", "latest_cached_frames_not_hardware_synchronized"},
        {"joint_order", "right.J1..J7,right.gripper,left.J1..J7,left.gripper"}, {"joint_units", "rad"}};
      for (size_t i = 0; i < ids_.size(); ++i) {
        if (!frames[i] || !fresh(frames[i]->receive_mono_ns, now, image_age_ns_)) {
          json_response(res, {{"error", "image_missing_or_stale"}, {"camera", ids_[i]}}, 503); return;
        }
        oldest = std::min(oldest, frames[i]->receive_mono_ns);
        newest = std::max(newest, frames[i]->receive_mono_ns);
        metadata["cameras"][ids_[i]] = frame_info(frames[i], now);
      }
      if (newest-oldest > skew_ns_) {
        json_response(res, {{"error", "image_skew_exceeded"}, {"skew_ms", (newest-oldest)/1e6}}, 503); return;
      }
      if (!state || !fresh(state->receive_mono_ns, now, state_age_ns_) ||
          !std::all_of(state->valid.begin(), state->valid.end(), [](bool v){return v;}) ||
          !std::all_of(state->position.begin(), state->position.end(), [](double v){return std::isfinite(v);})) {
        json_response(res, {{"error", "state_missing_invalid_or_stale"}}, 503); return;
      }
      metadata["image_skew_ms"] = (newest-oldest)/1e6;
      metadata["observation_mono_ns"] = std::to_string(std::min(oldest, state->receive_mono_ns));
      metadata["state"] = {{"position", state->position}, {"valid", state->valid},
        {"receive_mono_ns", std::to_string(state->receive_mono_ns)}, {"enabled", state->enabled}};
      // Hold immutable shared pointers for the entire response: metadata describes these exact JPEGs.
      const std::string boundary = "luxi_act_" + std::to_string(now);
      std::string body;
      auto part = [&](const std::string& name, const std::string& type, const char* data, size_t size) {
        body += "--"+boundary+"\r\nContent-Type: "+type+"\r\nContent-Disposition: form-data; name=\""+name+"\"\r\n\r\n";
        body.append(data, size); body += "\r\n";
      };
      const auto text = metadata.dump(); part("metadata", "application/json", text.data(), text.size());
      for (size_t i=0; i<ids_.size(); ++i)
        part(ids_[i], "image/jpeg", reinterpret_cast<const char*>(frames[i]->data.data()), frames[i]->data.size());
      body += "--"+boundary+"--\r\n";
      res.set_header("Cache-Control", "no-store");
      res.set_content(body, ("multipart/form-data; boundary="+boundary).c_str());
    });
    if (!http_.bind_to_port(host.c_str(), port)) throw std::runtime_error("HTTP bind failed");
    thread_ = std::thread([this]{ http_.listen_after_bind(); });
    RCLCPP_INFO(get_logger(), "Read-only ACT RGB HTTP listening on %s:%d", host.c_str(), static_cast<int>(port));
  }
  ~RgbHttp() override { http_.stop(); if(thread_.joinable()) thread_.join(); }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { auto node = std::make_shared<RgbHttp>(); rclcpp::spin(node); }
  catch(const std::exception& e) { fprintf(stderr, "%s\n", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
