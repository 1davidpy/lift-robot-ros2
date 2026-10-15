// SPDX-License-Identifier: Apache-2.0
// Deliberately excludes ROS: official SDK builds can bundle incompatible DDS.
#include <luxi/depth_source.hpp>
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <thread>

namespace {
// Serialize USB enumeration/startup across camera processes. Release as soon
// as the first complete sample arrives; steady-state capture remains parallel.
class StartupLock {
  int fd_ = -1;
 public:
  StartupLock() {
    auto path = "/tmp/luxi-realsense-startup-" + std::to_string(getuid()) + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0) throw std::runtime_error("Cannot open RealSense startup lock");
    const auto fail = [this](const char* error) {
      close(fd_); fd_ = -1; throw std::runtime_error(error);
    };
    struct stat value{};
    if (fstat(fd_, &value) || !S_ISREG(value.st_mode) || value.st_uid != getuid())
      fail("Invalid RealSense startup lock owner/type");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (flock(fd_, LOCK_EX | LOCK_NB)) {
      if ((errno != EWOULDBLOCK && errno != EAGAIN) || std::chrono::steady_clock::now() >= deadline)
        fail("Another RealSense camera is initializing; retry after it is streaming");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  void release() noexcept { if (fd_ >= 0) { close(fd_); fd_ = -1; } }
  ~StartupLock() { release(); }
};
class Source : public luxi::DepthSource {
  StartupLock startup_lock_;
  std::unique_ptr<rs2::context> context_;
  rs2::sensor sensor_;
  rs2::frame_queue queue_{1};
  rs2_intrinsics intr_{};
  std::unique_ptr<rs2::pipeline> pipeline_;
  rs2::align align_{RS2_STREAM_COLOR};
  bool opened_ = false, started_ = false;
  void close() noexcept {
    if (pipeline_ && started_) try { pipeline_->stop(); } catch (...) {}
    if (pipeline_) { started_ = false; pipeline_.reset(); }
    if (started_) try { sensor_.stop(); } catch (...) {}
    if (opened_) try { sensor_.close(); } catch (...) {}
    started_ = opened_ = false;
  }
 public:
  Source(const char* serial, int width, int height, int fps, bool aligned) {
    try {
      context_ = std::make_unique<rs2::context>("{\"dds\":{\"enabled\":false}}");
      rs2::device device;
      std::string found;
      auto devices = context_->query_devices();
      for (auto candidate : devices) {
        if (!candidate.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
        auto actual = std::string(candidate.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        auto asic = candidate.supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)
            ? std::string(candidate.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) : std::string();
        found += actual + "(ASIC/USB:" + asic + ") ";
        if (std::string(serial) == actual || std::string(serial) == asic) {
          if (device) throw std::runtime_error("Ambiguous RealSense serial; refusing to choose a camera");
          device = candidate;
        }
      }
      if (!device) throw std::runtime_error(std::string("RealSense serial not connected: ") + serial + "; SDK serials: " + found);
      if (aligned) {
        // Own BOTH sensors. A separate V4L2 RGB process must be stopped first.
        pipeline_ = std::make_unique<rs2::pipeline>(*context_);
        rs2::config config;
        config.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
        pipeline_->start(config); started_ = true;
        return;
      }
      rs2::stream_profile profile;
      for (const auto& candidate : device.query_sensors()) {
        if (!candidate.is<rs2::depth_sensor>()) continue;
        for (auto p : candidate.get_stream_profiles()) {
          auto v = p.as<rs2::video_stream_profile>();
          if (v && p.stream_type() == RS2_STREAM_DEPTH && p.format() == RS2_FORMAT_Z16 &&
              p.fps() == fps && v.width() == width && v.height() == height) {
            sensor_ = candidate; profile = p; break;
          }
        }
        if (sensor_) break;
      }
      if (!sensor_) throw std::runtime_error("Requested Z16 depth profile unavailable");
      intr_ = profile.as<rs2::video_stream_profile>().get_intrinsics();
      sensor_.open(profile); opened_ = true;
      sensor_.start(queue_); started_ = true;
    } catch (...) { close(); throw; }
  }
  ~Source() override { close(); }
  int poll(luxi::DepthSample& sample, char* error, size_t capacity) noexcept override {
    try {
      rs2::frame frame;
      if (pipeline_) {
        rs2::frameset frames;
        if (!pipeline_->try_wait_for_frames(&frames, 200)) return 0;
        auto aligned = align_.process(frames);
        auto color = aligned.get_color_frame();
        auto depth = aligned.get_depth_frame();
        if (!color || !depth) return 0;
        if (color.get_profile().format() != RS2_FORMAT_RGB8 ||
            color.get_width() != depth.get_width() || color.get_height() != depth.get_height() ||
            color.get_stride_in_bytes() < color.get_width() * 3)
          throw std::runtime_error("Malformed aligned RGB frame");
        sample.color.resize(size_t(color.get_width()) * color.get_height() * 3);
        auto data = static_cast<const uint8_t*>(color.get_data());
        for (int y = 0; y < color.get_height(); ++y)
          std::memcpy(sample.color.data() + size_t(y) * color.get_width() * 3,
                      data + size_t(y) * color.get_stride_in_bytes(), size_t(color.get_width()) * 3);
        sample.color_sequence = color.get_frame_number();
        sample.color_device_ms = color.get_timestamp();
        sample.color_clock_domain = rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain());
        sample.aligned_to_color = true;
        intr_ = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        frame = depth;
      } else if (!queue_.try_wait_for_frame(&frame, 200)) return 0;
      auto depth = frame.as<rs2::depth_frame>();
      if (!depth) return 0;
      const int width = depth.get_width(), height = depth.get_height(), stride = depth.get_stride_in_bytes();
      if (width <= 0 || height <= 0 || width > 1920 || height > 1080 || stride < width * 2)
        throw std::runtime_error("Malformed SDK depth dimensions/stride");
      sample.width = width; sample.height = height;
      sample.pixels.resize(size_t(width) * height);
      auto data = static_cast<const uint8_t*>(depth.get_data());
      for (int y = 0; y < height; ++y)
        std::memcpy(sample.pixels.data() + size_t(y) * width, data + size_t(y) * stride, size_t(width) * 2);
      sample.units = depth.get_units(); sample.sequence = depth.get_frame_number();
      sample.device_ms = depth.get_timestamp();
      sample.clock_domain = rs2_timestamp_domain_to_string(depth.get_frame_timestamp_domain());
      sample.fx = intr_.fx; sample.fy = intr_.fy; sample.cx = intr_.ppx; sample.cy = intr_.ppy;
      std::memcpy(sample.distortion, intr_.coeffs, sizeof(sample.distortion));
      const bool zero_distortion = std::all_of(intr_.coeffs, intr_.coeffs + 5,
          [](float value) { return value == 0.f; });
      sample.distortion_model = (zero_distortion || intr_.model == RS2_DISTORTION_NONE ||
          intr_.model == RS2_DISTORTION_BROWN_CONRADY) ? "plumb_bob" :
          "realsense_" + std::to_string(static_cast<int>(intr_.model));
      sample.calibrated = true;
      startup_lock_.release();
      return 1;
    } catch (const std::exception& e) { std::snprintf(error, capacity, "%s", e.what()); return -1; }
    catch (...) { std::snprintf(error, capacity, "Unknown depth SDK error"); return -1; }
  }
};
}
extern "C" luxi::DepthSource* luxi_create_depth_source_v3(const char* serial, int width, int height,
    int fps, bool aligned, char* error, size_t capacity) noexcept {
  try { return new Source(serial, width, height, fps, aligned); }
  catch (const std::exception& e) { std::snprintf(error, capacity, "%s", e.what()); return nullptr; }
  catch (...) { std::snprintf(error, capacity, "Unknown depth SDK initialization error"); return nullptr; }
}
