// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
// clang-format off
// Eigen must precede the _res resolver macro exposed by httplib on Ubuntu 22.04.
#include <Eigen/Core>
#include <httplib.h>
// clang-format on
#include <arpa/inet.h>

#include <sqlite3.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>


#include <atomic>
#include <set>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <luxi/ik.hpp>
#include <luxi/storage.hpp>
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <thread>
using namespace std::chrono_literals;
void require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
luxi::Json raw_pose(const geometry_msgs::msg::Pose& p) {
  Eigen::Matrix3d m;
  m << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  Eigen::Vector3d x = m.transpose() * (luxi::pos(p) - Eigen::Vector3d(-.085, 0, -.14));
  Eigen::Quaterniond q = Eigen::Quaterniond(m).conjugate() * luxi::rot(p) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
  return {{"x", x.x()},   {"y", x.y()},  {"z", -x.z()}, {"qx", -q.x()},
          {"qy", -q.y()}, {"qz", q.z()}, {"qw", q.w()}};
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::atomic<bool> running{true}, sending{true};
  std::thread spin, send;
  int fd = -1;
  int result = 1;
  std::mutex mutex;
  std::string phase;
  uint64_t quest_seq = 0;
  std::array<bool, 3> images{};
  sockaddr_in addr{};
  try {
    require(argc >= 3, "probe ROOT DATA_DIRECTORY");
    std::string root = argv[1];
    luxi::fs::path data = argv[2];
    auto node = std::make_shared<rclcpp::Node>("integration_probe");
    auto ctl = node->create_client<luxi::Service>("/luxi/control"),
         rec = node->create_client<luxi::Service>("/luxi/episode");

    auto states = node->create_subscription<std_msgs::msg::String>(
        "/luxi/control_status", rclcpp::QoS(1).transient_local(),
        [&](std_msgs::msg::String::ConstSharedPtr s) {
          std::lock_guard<std::mutex> l(mutex);
          phase = luxi::Json::parse(s->data).at("phase");
        });
    auto quests = node->create_subscription<luxi::Quest>("/luxi/quest", 100,
                                                         [&](luxi::Quest::ConstSharedPtr q) {
                                                           std::lock_guard<std::mutex> l(mutex);
                                                           quest_seq = q->sequence;
                                                         });
    std::vector<rclcpp::Subscription<luxi::Frame>::SharedPtr> cams;
    for (int i = 0; i < 3; i++) {
      std::string name = i == 0 ? "head_rgb" : i == 1 ? "left_wrist_rgb" : "right_wrist_rgb";
      cams.push_back(node->create_subscription<luxi::Frame>(
          "/luxi/cameras/" + name, 4, [&, i](luxi::Frame::ConstSharedPtr f) {
            std::lock_guard<std::mutex> l(mutex);
            images[i] = f->width == 640 && f->height == 360 && !f->data.empty() &&
                        f->timestamp_quality == "mock_host";
          }));
    }
    spin = std::thread([node] { rclcpp::spin(node); });
    auto request = [&](auto client, const std::string& command, const std::string& detail = "") {
      require(client->wait_for_service(10s), "service unavailable");
      auto r = std::make_shared<luxi::Service::Request>();
      r->command = command;
      r->detail = detail;
      auto f = client->async_send_request(r);
      require(f.wait_for(15s) == std::future_status::ready, "service timeout");
      auto response = f.get();
      require(response->success, "service rejected " + command + ": " + response->message);
    };
    luxi::ArmConfig cfg(luxi::config(root + "/Project/robot_bringup/config/arms.json"));
    luxi::Ik ik(root + "/device/openarm/openarm_mujoco/v2/cell/cell.xml", cfg);
    auto left = raw_pose(ik.fk(1)), right = raw_pose(ik.fk(0));
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    require(fd >= 0, "UDP socket");

    addr.sin_family = AF_INET;
    addr.sin_port = htons(15006);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    send = std::thread([&, left, right] {
      uint64_t n = 0;
      while (running) {
        if (sending) {
          luxi::Json q = {{"t", ++n / 72.0}, {"lc", left}, {"rc", right}, {"lt", 1.0},
                          {"rt", 1.0},       {"v", 0},     {"vl", 0},     {"vr", 0}};
          auto raw = q.dump();
          sendto(fd, raw.data(), raw.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        }
        std::this_thread::sleep_for(13889us);
      }
    });
    for (int i = 0; i < 150; ++i) {
      std::this_thread::sleep_for(100ms);
      std::lock_guard<std::mutex> l(mutex);
      if (phase == "disabled" && images[0] && images[1] && images[2] && quest_seq >= 100) break;
    }
    {
      std::lock_guard<std::mutex> l(mutex);
      require(phase == "disabled", "default must not enable motors");
      require(images[0] && images[1] && images[2], "three camera streams");
      require(quest_seq >= 50, "Quest rate");
    }
    request(ctl, "start");
    bool aligned = false;
    for (int i = 0; i < 200; i++) {
      std::this_thread::sleep_for(100ms);
      std::lock_guard<std::mutex> l(mutex);
      if (phase == "teleop") {
        aligned = true;
        break;
      }
    }
    require(aligned, "alignment failed: " + phase);
    request(rec, "start", "integration_capture");
    std::this_thread::sleep_for(2500ms);
    request(rec, "success");
    httplib::Client client("127.0.0.1", 18000);
    auto list = client.Get("/api/episodes");
    require(list && list->status == 200, "LAN episode list");
    auto ids = luxi::Json::parse(list->body);
    require(ids.size() == 1, "one episode");
    std::string id = ids[0];
    auto mres = client.Get(("/api/episodes/" + id + "/manifest").c_str());
    require(mres && mres->status == 200, "manifest HTTP");
    auto manifest = luxi::Json::parse(mres->body);
    require(manifest.at("samples").get<int>() > 10, "aligned samples absent");
    require(manifest.at("status") == "accepted", "episode QC: " + manifest.at("qc").dump());
    std::ifstream index(data / id / "samples.jsonl");
    std::string row;
    uint64_t rows = 0;
    std::set<std::string> referenced_frames;
    while (std::getline(index, row)) {
      const auto sample = luxi::Json::parse(row);
      auto tx = std::stoll(sample.at("tx_mono_ns").get<std::string>());
      auto state_time = std::stoll(sample.at("state_mono_ns").get<std::string>());
      require(state_time <= tx && tx - state_time <= 150000000, "state must precede action");
      require(sample.at("observation.state").size() == 16 && sample.at("action").size() == 16, "state/action dimensions");
      require(sample.at("images").size() == 3, "sample needs three camera references");
      for (const auto& image : sample.at("images")) {
        referenced_frames.insert(image.at("topic").get<std::string>() + ":" + std::to_string(image.at("frame_id").get<uint64_t>()));
        auto image_time = std::stoll(image.at("receive_mono_ns").get<std::string>());
        require(image_time <= tx && tx - image_time <= 200000000, "image must precede action");
      }
      ++rows;
    }
    require(rows == manifest.at("samples").get<uint64_t>(), "manifest/sample row count");
    {
      rosbag2_cpp::Reader reader;
      reader.open((data / id / "bag").string());
      rclcpp::Serialization<luxi::Frame> serialization;
      while (reader.has_next()) {
        auto raw = reader.read_next();
        if (raw->topic_name.rfind("/luxi/cameras/", 0) != 0) continue;
        rclcpp::SerializedMessage serialized(*raw->serialized_data);
        luxi::Frame frame;
        serialization.deserialize_message(&serialized, &frame);
        referenced_frames.erase(raw->topic_name + ":" + std::to_string(frame.sequence));
      }
      require(referenced_frames.empty(), "every indexed frame must exist in the episode bag");
    }
    auto entries = manifest.at("files");
    require(entries.size() >= 4, "bag artifacts missing");
    std::string first = entries[0].at("path");
    auto range =
        client.Get(("/api/files/" + id + "/" + first).c_str(), httplib::Headers{{"Range", "bytes=0-6"}});
    require(range && range->status == 206 && range->body.size() == 7, "HTTP byte range");
    auto bad = client.Get(("/api/files/" + id + "/../../README.md").c_str());
    require(bad && bad->status != 200, "path traversal");
    auto dst = data.parent_path() / "received";
    auto partial = dst / (".partial-" + id) / first;
    luxi::fs::create_directories(partial.parent_path());
    {
      std::ofstream out(partial, std::ios::binary);
      out.write(range->body.data(), range->body.size());
    }
    std::string exe =
        root + "/build/install/luxi_data_recorder/lib/luxi_data_recorder/episode_pull";
    pid_t pid = fork();
    require(pid >= 0, "fork download");
    if (pid == 0) {
      execl(exe.c_str(), exe.c_str(), "http://127.0.0.1:18000", id.c_str(), dst.c_str(), nullptr);
      _exit(127);
    }
    int child;
    waitpid(pid, &child, 0);
    require(WIFEXITED(child) && WEXITSTATUS(child) == 0, "resumable downloader");
    for (const auto& f : entries)
      require(luxi::hash_file(dst / id / f.at("path").get<std::string>()) == f.at("sha256"),
              "download hash");
    for (const auto& f : entries) {
      auto name = f.at("path").get<std::string>();
      if (luxi::fs::path(name).extension() == ".db3") {
        sqlite3* db = nullptr;
        require(sqlite3_open((data / id / name).c_str(), &db) == SQLITE_OK, "bag sqlite");
        sqlite3_stmt* stmt = nullptr;
        require(sqlite3_prepare_v2(db, "SELECT count(*) FROM messages", -1, &stmt, nullptr) ==
                    SQLITE_OK,
                "bag messages table");
        require(sqlite3_step(stmt) == SQLITE_ROW && sqlite3_column_int(stmt, 0) > 100,
                "bag message count");
        sqlite3_finalize(stmt);
        sqlite3_close(db);
      }
    }
    sending = false;
    std::this_thread::sleep_for(400ms);
    uint64_t stopped;
    {
      std::lock_guard<std::mutex> l(mutex);
      stopped = quest_seq;
      require(phase == "hold", "UDP loss must enter hold");
    }
    std::this_thread::sleep_for(250ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      require(stopped == quest_seq, "receiver replayed cached packet");
    }
    sending = true;
    std::this_thread::sleep_for(400ms);
    {
      std::lock_guard<std::mutex> l(mutex);
      require(phase == "hold", "reconnect must not automatically resume");
    }
    request(ctl, "stop");
    for (int i = 0; i < 150; ++i) {
      std::this_thread::sleep_for(100ms);
      std::lock_guard<std::mutex> l(mutex);
      if (phase == "disabled" && images[0] && images[1] && images[2] && quest_seq >= 100) break;
    }
    {
      std::lock_guard<std::mutex> l(mutex);
      require(phase == "disabled", "return-to-home pause");
    }
    std::cout << "PASS: UDP -> mapping -> C++ IK -> mock OpenArm -> three cameras -> rosbag/sample "
                 "-> HTTP range -> resumed hash-verified download; loss/reconnect/pause\n";
    result = 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
  }
  running = false;
  if (send.joinable()) send.join();
  if (fd >= 0) close(fd);
  rclcpp::shutdown();
  if (spin.joinable()) spin.join();
  return result;
}
