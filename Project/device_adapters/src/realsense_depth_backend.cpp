// SPDX-License-Identifier: Apache-2.0
// Deliberately excludes ROS: official SDK builds can bundle incompatible DDS.
#include <luxi/depth_source.hpp>
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace {
class Source : public luxi::DepthSource {
  rs2::context context_{"{\"dds\":{\"enabled\":false}}"};
  rs2::sensor sensor_;
  rs2::frame_queue queue_{1};
  rs2_intrinsics intr_{};
  bool opened_ = false, started_ = false;
  void close() noexcept {
    if (started_) try { sensor_.stop(); } catch (...) {}
    if (opened_) try { sensor_.close(); } catch (...) {}
    started_ = opened_ = false;
  }
 public:
  Source(const char* serial, int width, int height, int fps) {
    try {
      rs2::device device;
      std::string found;
      auto devices = context_.query_devices();
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
      if (!queue_.try_wait_for_frame(&frame, 200)) return 0;
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
      sample.calibrated = intr_.model == RS2_DISTORTION_NONE || intr_.model == RS2_DISTORTION_BROWN_CONRADY;
      return 1;
    } catch (const std::exception& e) { std::snprintf(error, capacity, "%s", e.what()); return -1; }
    catch (...) { std::snprintf(error, capacity, "Unknown depth SDK error"); return -1; }
  }
};
}
extern "C" luxi::DepthSource* luxi_create_depth_source(const char* serial, int width, int height,
    int fps, char* error, size_t capacity) noexcept {
  try { return new Source(serial, width, height, fps); }
  catch (const std::exception& e) { std::snprintf(error, capacity, "%s", e.what()); return nullptr; }
  catch (...) { std::snprintf(error, capacity, "Unknown depth SDK initialization error"); return nullptr; }
}
