// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <deque>
#include <luxi/storage.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>
class Recorder : public rclcpp::Node {
  luxi::fs::path root_, active_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::ofstream samples_;
  luxi::Json counts_, qc_ = luxi::Json::array();
  std::string id_, task_, phase_ = "idle";
  bool require_cameras_;
  uint64_t count_ = 0;
  int64_t last_sample_ = 0;
  double reserve_;
  luxi::State state_;
  std::deque<luxi::State> state_history_;
  std::array<std::deque<luxi::Frame>, 3> frame_history_;
  luxi::Json hardware_config_;
  std::array<luxi::Frame, 3> frames_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<luxi::Service>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_, shutdown_;
  rclcpp::Client<luxi::Service>::SharedPtr control_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<bool, 4> buttons_{};
  int64_t last_quest_ = 0, y_start_ = 0, b_start_ = 0;
  bool menu_ = false, y_fired_ = false, b_fired_ = false, quit_pending_ = false;
  const std::array<std::string, 5> reasons_{{"bad_demonstration", "grasp_failure",
                                             "incomplete_fold", "unsafe_motion",
                                             "hardware_or_sensor"}};
  int reason_ = 0;
  int64_t menu_time_ = 0;
  void status() {
    std_msgs::msg::String m;
    m.data = luxi::Json{
        {"state", phase_},
        {"episode", id_},
        {"samples", count_},
        {"failure_menu", menu_},
        {"failure_reason_index", reason_},
        {"qc",
         qc_}}.dump();
    status_->publish(m);
  }
  template <class T>
  void save(const std::string& topic, const T& m) {
    if (!writer_) return;
    writer_->write(m, topic, now());
    counts_[topic] = counts_.value(topic, 0) + 1;
  }
  template <class T>
  void watch(const std::string& topic, std::function<void(const T&)> extra = {}) {
    subscriptions_.push_back(create_subscription<T>(
        topic, rclcpp::QoS(100).reliable(), [this, topic, extra](typename T::ConstSharedPtr m) {
          try {
            save(topic, *m);
            if (extra) extra(*m);
          } catch (const std::exception& e) {
            qc_.push_back(e.what());
            RCLCPP_ERROR(get_logger(), "recording: %s", e.what());
          }
        }));
  }
  void pause() {
    if (control_->service_is_ready()) {
      auto r = std::make_shared<luxi::Service::Request>();
      r->command = "stop";
      control_->async_send_request(r);
    }
  }
  bool start(const std::string& task) {
    if (writer_ || task.empty() || !luxi::valid_state(state_) || !state_.enabled ||
        !luxi::fresh(state_.receive_mono_ns, luxi::mono()))
      return false;
    if (require_cameras_)
      for (const auto& f : frames_)
        if (f.data.empty() || !luxi::fresh(f.receive_mono_ns, luxi::mono(), 200000000))
          return false;
    if (luxi::fs::space(root_).available < reserve_ * 1024 * 1024 * 1024) return false;
    id_ = "episode-" + std::to_string(now().nanoseconds());
    active_ = root_ / (".partial-" + id_);
    luxi::fs::create_directory(active_);
    task_ = task;
    counts_ = luxi::Json::object();
    qc_ = luxi::Json::array();
    count_ = 0;
    last_sample_ = 0;
    luxi::atomic_json(active_ / "session.json",
                      {{"schema", "luxi_arms_v1"},
                       {"order", "right[8],left[8]"},
                       {"joint_units", "rad"},
                       {"gripper_units", "signed_logical_action_motor_feedback"},
                       {"task", task},
                       {"started_ns", std::to_string(now().nanoseconds())},
                       {"time_domain", "board_monotonic_and_ros_wall"}});
    luxi::atomic_json(active_ / "robot_config.json", hardware_config_);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open((active_ / "bag").string());
    // Include the initial observation context so early sample references always
    // point to messages that are actually present in this episode's bag.
    state_history_.clear();
    state_history_.push_back(state_);
    save("/luxi/arms/state", state_);
    for (size_t i = 0; i < frames_.size(); ++i) {
      frame_history_[i].clear();
      if (frames_[i].data.empty()) continue;
      frame_history_[i].push_back(frames_[i]);
      save("/luxi/cameras/" + frames_[i].camera_id, frames_[i]);
    }
    samples_.open(active_ / "samples.jsonl");
    if (!samples_) throw std::runtime_error("samples open");
    phase_ = "recording";
    status();
    return true;
  }
  bool finish(const std::string& result, const std::string& detail) {
    if (!writer_) return false;
    phase_ = "finalizing";
    writer_.reset();
    samples_.flush();
    samples_.close();
    if (count_ == 0) qc_.push_back("no aligned samples");
    for (const auto& topic : {"/luxi/arms/state", "/luxi/applied"})
      if (counts_.value(topic, 0) == 0) qc_.push_back(std::string("missing ") + topic);
    if (require_cameras_)
      for (const auto& c : {"head_rgb", "left_wrist_rgb", "right_wrist_rgb"})
        if (counts_.value(std::string("/luxi/cameras/") + c, 0) == 0)
          qc_.push_back(std::string("missing ") + c);
    auto files = luxi::manifest_files(active_);
    std::string label = result == "success" && qc_.empty() ? "accepted"
                        : result == "cancel"               ? "cancelled"
                                                           : "failed";
    luxi::atomic_json(active_ / "manifest.json", {{"episode_id", id_},
                                                  {"status", label},
                                                  {"task", task_},
                                                  {"detail", detail},
                                                  {"samples", count_},
                                                  {"counts", counts_},
                                                  {"qc", qc_},
                                                  {"files", files}});
    for (const auto& e : luxi::fs::recursive_directory_iterator(active_))
      if (e.is_directory()) luxi::sync_path(e.path());
    luxi::sync_path(active_);
    luxi::fs::rename(active_, root_ / id_);
    luxi::sync_path(root_);
    phase_ = "idle";
    menu_ = false;
    status();
    return true;
  }
  void sample(const luxi::Command& c) {
    if (!writer_ || c.source_mono_ns - last_sample_ < 30000000) return;
    last_sample_ = c.source_mono_ns;
    if (c.phase != "teleop") return;
    const luxi::State* observation = nullptr;
    for (auto it = state_history_.rbegin(); it != state_history_.rend(); ++it)
      if (it->receive_mono_ns <= c.source_mono_ns) {
        observation = &*it;
        break;
      }
    bool valid = observation && luxi::valid_state(*observation) &&
                 luxi::fresh(observation->receive_mono_ns, c.source_mono_ns);
    luxi::Json images = luxi::Json::object();
    for (const auto& history : frame_history_) {
      const luxi::Frame* selected = nullptr;
      for (auto it = history.rbegin(); it != history.rend(); ++it)
        if (it->receive_mono_ns <= c.source_mono_ns) {
          selected = &*it;
          break;
        }
      if (!selected || !luxi::fresh(selected->receive_mono_ns, c.source_mono_ns, 200000000) ||
          selected->data.empty()) {
        if (require_cameras_) valid = false;
        continue;
      }
      const auto& f = *selected;
      images[f.camera_id] = {{"frame_id", f.sequence},
                             {"receive_mono_ns", std::to_string(f.receive_mono_ns)},
                             {"device_timestamp", std::to_string(f.device_timestamp)},
                             {"timestamp_quality", f.timestamp_quality},
                             {"topic", "/luxi/cameras/" + f.camera_id}};
    }
    if (!valid) {
      qc_.push_back("invalid sample at " + std::to_string(c.source_mono_ns));
      return;
    }
    samples_ << luxi::Json{{"sample_id", count_++},
                           {"observation.state", observation->position},
                           {"observation.velocity", observation->velocity},
                           {"action", c.position},
                           {"action_sequence", c.sequence},
                           {"tx_mono_ns", std::to_string(c.source_mono_ns)},
                           {"state_mono_ns", std::to_string(observation->receive_mono_ns)},
                           {"images", images}}
                    .dump()
             << '\n';
    if (!samples_) throw std::runtime_error("sample write failed");
  }

 public:
  Recorder() : Node("data_recorder") {
    auto config_file = declare_parameter<std::string>("config", "");
    if (!config_file.empty()) hardware_config_ = luxi::config(config_file);
    root_ = declare_parameter<std::string>("directory", "data");
    reserve_ = declare_parameter("minimum_free_gib", 20.0);
    require_cameras_ = declare_parameter("require_cameras", true);
    task_ = declare_parameter("task", "teleoperation");
    luxi::fs::create_directories(root_);
    // Never merge interrupted episodes into the accepted tree; preserve them for bag recovery.
    for (const auto& e : luxi::fs::directory_iterator(root_))
      if (e.is_directory() && e.path().filename().string().rfind(".partial-", 0) == 0)
        RCLCPP_WARN(get_logger(), "Interrupted episode retained: %s", e.path().c_str());
    status_ = create_publisher<std_msgs::msg::String>("/luxi/recorder/status",
                                                      rclcpp::QoS(1).transient_local());
    shutdown_ = create_publisher<std_msgs::msg::String>("/luxi/shutdown", 10);
    control_ = create_client<luxi::Service>("/luxi/control");
    watch<luxi::State>("/luxi/arms/state", [this](const auto& s) {
      state_ = s;
      state_history_.push_back(s);
      if (state_history_.size() > 64) state_history_.pop_front();
    });
    watch<luxi::Command>("/luxi/applied", [this](const auto& c) { sample(c); });
    watch<luxi::Command>("/luxi/intent");
    watch<luxi::Command>("/luxi/command");
    watch<luxi::Targets>("/luxi/targets");
    watch<std_msgs::msg::String>("/luxi/control_status");
    watch<std_msgs::msg::String>("/luxi/faults", [this](const auto& s) {
      if (writer_) qc_.push_back(s.data);
    });
    for (int i = 0; i < 3; i++) {
      std::string name = i == 0 ? "head_rgb" : i == 1 ? "left_wrist_rgb" : "right_wrist_rgb";
      watch<luxi::Frame>("/luxi/cameras/" + name, [this, i](const auto& f) {
        if (writer_ && frames_[i].receive_mono_ns) {
          if (f.sequence == frames_[i].sequence || f.receive_mono_ns <= frames_[i].receive_mono_ns)
            qc_.push_back("camera duplicate/backwards");
        }
        frames_[i] = f;
        frame_history_[i].push_back(f);
        if (frame_history_[i].size() > 16) frame_history_[i].pop_front();
      });
    }
    watch<luxi::Quest>("/luxi/quest", [this](const auto& q) {
      if (!luxi::fresh(q.receive_mono_ns, luxi::mono())) return;
      if (!luxi::fresh(last_quest_, q.receive_mono_ns, 500000000)) {
        buttons_.fill(false);
        y_start_ = 0;
        b_start_ = 0;
      }
      last_quest_ = q.receive_mono_ns;
      if (q.buttons[0] && !buttons_[0]) {
        if (menu_)
          finish("fail", reasons_[reason_]);
        else if (writer_)
          finish("success", "");
        else
          start(task_);
      }
      if (q.buttons[1] && !buttons_[1]) {
        b_start_ = q.receive_mono_ns;
        b_fired_ = false;
        if (writer_) {
          menu_ = true;
          pause();
        }
      }
      if (menu_ && std::abs(q.sticks[1]) >= .7 && q.receive_mono_ns - menu_time_ > 350000000) {
        reason_ = (reason_ + (q.sticks[1] < 0 ? 1 : 4)) % 5;
        menu_time_ = q.receive_mono_ns;
      }
      if (q.buttons[3] && !buttons_[3]) {
        y_start_ = q.receive_mono_ns;
        y_fired_ = false;
      }
      if (q.buttons[3] && !y_fired_ && q.receive_mono_ns - y_start_ >= 1500000000) {
        finish("cancel", "operator");
        y_fired_ = true;
      }
      if (q.buttons[1] && !b_fired_ && !writer_ && q.receive_mono_ns - b_start_ >= 1500000000) {
        pause();
        quit_pending_ = true;
        b_fired_ = true;
      }
      buttons_ = q.buttons;
    });
    service_ = create_service<luxi::Service>(
        "/luxi/episode", [this](const std::shared_ptr<luxi::Service::Request> r,
                                std::shared_ptr<luxi::Service::Response> s) {
          try {
            s->success =
                r->command == "start" ? start(r->detail.empty() ? task_ : r->detail)
                : (r->command == "success" || r->command == "fail" || r->command == "cancel")
                    ? finish(r->command, r->detail)
                    : false;
            s->message = phase_;
          } catch (const std::exception& e) {
            s->success = false;
            s->message = e.what();
            qc_.push_back(e.what());
            phase_ = "fault";
          }
          status();
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      try {
        if (writer_) {
          samples_.flush();
          if (luxi::fs::space(root_).available < reserve_ * 1024 * 1024 * 1024)
            finish("fail", "disk_space");
        }
        if (quit_pending_ && !state_.enabled && luxi::fresh(state_.receive_mono_ns, luxi::mono())) {
          std_msgs::msg::String quit;
          quit.data = "operator_quit";
          shutdown_->publish(quit);
          quit_pending_ = false;
        }
        status();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
      }
    });
  }
  ~Recorder() {
    try {
      if (writer_) finish("cancel", "shutdown");
    } catch (...) {
    }
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<Recorder>(); });
}
