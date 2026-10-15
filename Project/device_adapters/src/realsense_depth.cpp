// SPDX-License-Identifier: Apache-2.0
// In aligned mode this process owns both color and depth sensors.
#include <luxi/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <atomic>
#include <cstring>
#include <limits>
#include <thread>
#include <filesystem>
#include <dlfcn.h>
#include <luxi/depth_source.hpp>
#include <luxi_interfaces/msg/rgbd_frame.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class DepthCamera : public rclcpp::Node {
  std::string id_, serial_;
  bool mock_, aligned_;
  int width_, height_, fps_, jpeg_quality_;
  std::string session_;
  std::atomic<bool> active_{true};
  std::thread worker_;
  rclcpp::Publisher<luxi::Frame>::SharedPtr frames_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<luxi_interfaces::msg::RGBDFrame>::SharedPtr pairs_;
  rclcpp::Publisher<luxi::Frame>::SharedPtr colors_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_status_;

  void status(const std::string& phase, const std::string& reason) {
    std_msgs::msg::String m;
    m.data = luxi::Json{{"camera_id", id_}, {"serial", serial_}, {"mock", mock_},
        {"phase", phase}, {"reason", reason}, {"encoding", "32FC1"},
        {"unit", "metre"}, {"aligned_to_color", aligned_},
        {"receive_mono_ns", std::to_string(luxi::mono())}}.dump();
    status_->publish(m);
    if (color_status_) {
      auto j = luxi::Json::parse(m.data);
      j["encoding"] = "jpeg"; j["width"] = width_; j["height"] = height_;
      m.data = j.dump(); color_status_->publish(m);
    }
  }
  void publish(const uint16_t* raw, int width, int height, int stride,
               float scale, uint64_t sequence, double device_ms,
               const std::string& quality, sensor_msgs::msg::CameraInfo info,
               const luxi::DepthSample* sample = nullptr) {
    if (!std::isfinite(scale) || scale <= 0 || width <= 0 || height <= 0 ||
        stride < width * 2) throw std::runtime_error("Invalid depth frame/units");
    luxi::Frame f;
    f.header.stamp = now(); f.header.frame_id = id_ + (aligned_ ? "_color_optical" : "_depth_optical");
    f.camera_id = id_; f.sequence = sequence; f.receive_mono_ns = luxi::mono();
    // SDK timestamp is milliseconds in the separately named SDK clock domain.
    f.device_timestamp = static_cast<int64_t>(device_ms * 1000000.0);
    f.timestamp_quality = quality;
    f.width = width; f.height = height; f.encoding = "32FC1";
    f.data.resize(size_t(width) * height * sizeof(float));
    for (int y = 0; y < height; ++y) {
      auto row = reinterpret_cast<const uint8_t*>(raw) + size_t(y) * stride;
      for (int x = 0; x < width; ++x) {
        uint16_t value;
        std::memcpy(&value, row + x * 2, sizeof(value));
        float metres = value ? value * scale : std::numeric_limits<float>::quiet_NaN();
        std::memcpy(f.data.data() + (size_t(y) * width + x) * 4, &metres, 4);
      }
    }
    info.header = f.header; info.width = width; info.height = height;
    if (aligned_) {
      if (!sample || !sample->aligned_to_color || sample->color.size() != size_t(width) * height * 3)
        throw std::runtime_error("Missing paired RGB for aligned depth");
      luxi_interfaces::msg::RGBDFrame pair;
      pair.header = f.header; pair.camera_id = id_; pair.sequence = sequence;
      pair.pair_id = session_ + ":" + std::to_string(sequence);
      pair.depth_device_sequence = sample->sequence;
      pair.color_device_sequence = sample->color_sequence;
      pair.depth = f; pair.camera_info = info;
      auto& color = pair.color;
      color.header = f.header; color.camera_id = id_; color.sequence = sequence;
      color.receive_mono_ns = f.receive_mono_ns;
      color.device_timestamp = static_cast<int64_t>(sample->color_device_ms * 1000000.0);
      color.timestamp_quality = mock_ ? "mock_host" : "host_receive_device_unmapped:" + sample->color_clock_domain;
      color.width = width; color.height = height; color.encoding = "jpeg";
      cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(sample->color.data()));
      cv::Mat bgr; cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      if (!cv::imencode(".jpg", bgr, color.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_}))
        throw std::runtime_error("RGB JPEG encoding failed");
      pairs_->publish(pair);
      colors_->publish(color); color_info_->publish(info);
      sensor_msgs::msg::CompressedImage compressed;
      compressed.header = color.header; compressed.format = "jpeg"; compressed.data = color.data;
      compressed_->publish(std::move(compressed));
    }
    // CameraInfo and frame share a stamp; the gateway only joins matching pairs.
    info_->publish(info);
    frames_->publish(f);
    sensor_msgs::msg::Image image;
    image.header = f.header; image.width = width; image.height = height;
    image.encoding = "32FC1"; image.is_bigendian = false; image.step = width * 4;
    image.data = std::move(f.data); images_->publish(std::move(image));
  }
  void capture() {
    uint64_t sequence = 0;
    while (active_ && rclcpp::ok()) {
      try {
        if (mock_) {
          std::vector<uint16_t> raw(size_t(width_) * height_, 1000);
          raw[0] = 0;
          sensor_msgs::msg::CameraInfo info;
          info.distortion_model = "plumb_bob"; info.d.assign(5, 0.0);
          info.k = {600., 0., width_ / 2., 0., 600., height_ / 2., 0., 0., 1.};
          info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
          info.p = {600., 0., width_ / 2., 0., 0., 600., height_ / 2., 0., 0., 0., 1., 0.};
          luxi::DepthSample sample;
          sample.aligned_to_color = aligned_;
          if (aligned_) {
            sample.color.resize(size_t(width_) * height_ * 3);
            for (size_t i = 0; i < sample.color.size(); i += 3) {
              sample.color[i] = 10; sample.color[i + 1] = 40; sample.color[i + 2] = 200;
            }
          }
          sample.sequence = sample.color_sequence = sequence + 1;
          publish(raw.data(), width_, height_, width_ * 2, .001f, ++sequence, 0, "mock_host", info, &sample);
          if ((sequence - 1) % fps_ == 0) status("streaming", "MOCK synthetic depth; not a physical measurement");
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_));
          continue;
        }
#ifdef LUXI_HAVE_REALSENSE
        struct Backend {
          void* library = nullptr;
          luxi::DepthSource* source = nullptr;
          ~Backend() { delete source; if (library) dlclose(library); }
        } backend;
        auto path = std::filesystem::canonical("/proc/self/exe").parent_path() / "libluxi_realsense_depth.so";
        // Vendor DDS stays local; its relocations must not bind ROS Humble's DDS.
        backend.library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND | RTLD_NODELETE);
        if (!backend.library) throw std::runtime_error(dlerror());
        auto create = reinterpret_cast<luxi::CreateDepthSource>(dlsym(backend.library, "luxi_create_depth_source_v3"));
        if (!create) throw std::runtime_error("Missing depth backend entry point");
        char error[1024]{};
        backend.source = create(serial_.c_str(), width_, height_, fps_, aligned_, error, sizeof(error));
        if (!backend.source) throw std::runtime_error(error);
        status("waiting", aligned_ ? "Waiting for RGB and depth-to-color aligned frameset" : "Waiting for native depth (not aligned to RGB)");
        auto last = luxi::mono(), report = last;
        uint64_t previous = 0; bool seen = false;
        while (active_ && rclcpp::ok()) {
          luxi::DepthSample sample;
          const int result = backend.source->poll(sample, error, sizeof(error));
          if (result < 0) throw std::runtime_error(error);
          if (result == 0) {
            if (luxi::mono() - last > 5000000000) throw std::runtime_error("Depth frame timeout");
            continue;
          }
          if (seen && sample.sequence == previous) continue;
          previous = sample.sequence; seen = true; last = luxi::mono();
          sensor_msgs::msg::CameraInfo info;
          if (sample.calibrated) {
            info.distortion_model = sample.distortion_model;
            info.d.assign(sample.distortion, sample.distortion + 5);
            info.k = {sample.fx, 0., sample.cx, 0., sample.fy, sample.cy, 0., 0., 1.};
            info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
            info.p = {sample.fx, 0., sample.cx, 0., 0., sample.fy, sample.cy, 0., 0., 0., 1., 0.};
          }
          publish(sample.pixels.data(), sample.width, sample.height, sample.width * 2,
              sample.units, ++sequence, sample.device_ms,
              "host_receive_device_unmapped:" + sample.clock_domain, info, &sample);
          if (last - report > 1000000000) {
            status("streaming", aligned_ ? "Paired RGB and depth aligned to color; invalid pixels are NaN" : "Native depth in metres; zero source pixels become NaN"); report = last;
          }
        }
#else
        throw std::runtime_error("Built without librealsense2; run tool/build_depth_sdk.sh and rebuild");
#endif
      } catch (const std::exception& e) {
        status("error", e.what()); RCLCPP_ERROR(get_logger(), "%s", e.what());
        for (int i = 0; i < 60 && active_ && rclcpp::ok(); ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }
 public:
  DepthCamera() : Node("realsense_depth") {
    id_ = declare_parameter<std::string>("camera_id", "head_rgb");
    serial_ = declare_parameter<std::string>("serial", "");
    mock_ = declare_parameter("mock", false);
    aligned_ = declare_parameter("align_to_color", true);
    jpeg_quality_ = declare_parameter("jpeg_quality", 90);
    session_ = std::to_string(luxi::mono());
    width_ = declare_parameter("width", 640); height_ = declare_parameter("height", 480);
    fps_ = declare_parameter("fps", 15);
    if ((id_ != "head_rgb" && id_ != "left_wrist_rgb" && id_ != "right_wrist_rgb") ||
        (!mock_ && (serial_.empty() || serial_.find_first_not_of("0123456789") != std::string::npos)) ||
        width_ < 16 || width_ > 1920 || height_ < 16 || height_ > 1080 || fps_ < 1 || fps_ > 60 ||
        jpeg_quality_ < 1 || jpeg_quality_ > 100)
      throw std::invalid_argument("Invalid depth camera parameters");
    const uint16_t endian = 1;
    if (*reinterpret_cast<const uint8_t*>(&endian) != 1)
      throw std::runtime_error("Depth wire format requires a little-endian host");
    frames_ = create_publisher<luxi::Frame>("/luxi/cameras/" + id_ + "/depth", rclcpp::SensorDataQoS());
    images_ = create_publisher<sensor_msgs::msg::Image>("/camera/" + id_ + "/depth/image_raw", rclcpp::SensorDataQoS());
    info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/" + id_ + "/depth/camera_info", rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("/luxi/cameras/" + id_ + "/depth/status", rclcpp::QoS(1).transient_local());
    if (aligned_) {
      pairs_ = create_publisher<luxi_interfaces::msg::RGBDFrame>("/luxi/cameras/" + id_ + "/rgbd", rclcpp::SensorDataQoS().keep_last(1));
      colors_ = create_publisher<luxi::Frame>("/luxi/cameras/" + id_, 1);
      compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/" + id_ + "/image/compressed", rclcpp::SensorDataQoS());
      color_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/" + id_ + "/color/camera_info", rclcpp::SensorDataQoS());
      color_status_ = create_publisher<std_msgs::msg::String>("/luxi/cameras/" + id_ + "/status", rclcpp::QoS(1).transient_local());
    }
    worker_ = std::thread([this] { capture(); });
  }
  ~DepthCamera() { active_ = false; if (worker_.joinable()) worker_.join(); }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<DepthCamera>(); });
}
