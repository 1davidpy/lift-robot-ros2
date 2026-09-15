// SPDX-License-Identifier: Apache-2.0
// Native UVC RGB acquisition; no librealsense dependency or depth stream.
#include <luxi/core.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <thread>

namespace {
int ioctl_retry(int fd, unsigned long request, void* arg) {
  int result;
  do { result = ioctl(fd, request, arg); } while (result < 0 && errno == EINTR);
  return result;
}
void require_ioctl(int fd, unsigned long request, void* arg, const char* label) {
  if (ioctl_retry(fd, request, arg) < 0)
    throw std::runtime_error(std::string(label) + ": " + std::strerror(errno));
}
std::string read_line(const std::filesystem::path& path) {
  std::ifstream f(path);
  std::string value;
  std::getline(f, value);
  return value;
}
std::string rgb_device(const std::string& serial) {
  const std::filesystem::path root("/sys/class/video4linux");
  if (!std::filesystem::exists(root)) throw std::runtime_error("No V4L2 cameras connected");
  for (const auto& entry : std::filesystem::directory_iterator(root)) {
    std::error_code ec;
    auto parent = std::filesystem::canonical(entry.path() / "device", ec);
    if (ec) continue;
    while (!parent.empty() && parent != parent.root_path() &&
           !std::filesystem::exists(parent / "idVendor")) parent = parent.parent_path();
    if (read_line(parent / "idVendor") != "8086" || read_line(parent / "serial") != serial)
      continue;
    const auto path = "/dev/" + entry.path().filename().string();
    int fd = open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) continue;
    v4l2_capability cap{};
    bool rgb = false;
    if (ioctl_retry(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      const auto caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
      if ((caps & V4L2_CAP_VIDEO_CAPTURE) && (caps & V4L2_CAP_STREAMING)) {
        v4l2_fmtdesc format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (; ioctl_retry(fd, VIDIOC_ENUM_FMT, &format) == 0; ++format.index)
          if (format.pixelformat == V4L2_PIX_FMT_YUYV) rgb = true;
      }
    }
    close(fd);
    if (rgb) return path;
  }
  throw std::runtime_error("RealSense RGB interface unavailable: " + serial);
}

class Capture {
  struct Mapping { void* data; size_t size; };
  int fd_ = -1;
  bool started_ = false, have_format_ = false, have_rate_ = false;
  std::vector<Mapping> buffers_;
  v4l2_format original_format_{};
  v4l2_streamparm original_rate_{};
  unsigned width_, height_, stride_ = 0;
  void cleanup() noexcept {
    if (fd_ < 0) return;
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (started_) ioctl_retry(fd_, VIDIOC_STREAMOFF, &type);
    for (auto& b : buffers_) munmap(b.data, b.size);
    buffers_.clear();
    v4l2_requestbuffers request{};
    request.type = type; request.memory = V4L2_MEMORY_MMAP;
    ioctl_retry(fd_, VIDIOC_REQBUFS, &request);
    if (have_format_) ioctl_retry(fd_, VIDIOC_S_FMT, &original_format_);
    if (have_rate_) ioctl_retry(fd_, VIDIOC_S_PARM, &original_rate_);
    close(fd_); fd_ = -1;
  }
 public:
  std::string device;
  Capture(const std::string& serial, unsigned width, unsigned height, unsigned fps)
      : width_(width), height_(height), device(rgb_device(serial)) {
    try {
      fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0) throw std::runtime_error("Camera open: " + std::string(std::strerror(errno)));
      original_format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      require_ioctl(fd_, VIDIOC_G_FMT, &original_format_, "Read original format"); have_format_ = true;
      original_rate_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      have_rate_ = ioctl_retry(fd_, VIDIOC_G_PARM, &original_rate_) == 0;
      v4l2_format format{};
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = width; format.fmt.pix.height = height;
      format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      format.fmt.pix.field = V4L2_FIELD_NONE;
      require_ioctl(fd_, VIDIOC_S_FMT, &format, "Set RGB format");
      if (format.fmt.pix.width != width || format.fmt.pix.height != height ||
          format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
        throw std::runtime_error("Camera does not support the requested YUYV resolution");
      stride_ = format.fmt.pix.bytesperline;
      if (stride_ < width * 2) throw std::runtime_error("Invalid camera stride");
      v4l2_streamparm rate{};
      rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      rate.parm.capture.timeperframe.numerator = 1;
      rate.parm.capture.timeperframe.denominator = fps;
      require_ioctl(fd_, VIDIOC_S_PARM, &rate, "Set frame rate");
      auto interval = rate.parm.capture.timeperframe;
      if (!interval.numerator || std::abs(double(interval.denominator) / interval.numerator - fps) > .1)
        throw std::runtime_error("Camera did not accept the requested frame rate");
      v4l2_requestbuffers request{};
      request.count = 4; request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; request.memory = V4L2_MEMORY_MMAP;
      require_ioctl(fd_, VIDIOC_REQBUFS, &request, "Request camera buffers");
      if (request.count < 2 || request.count > 32) throw std::runtime_error("Invalid camera buffer count");
      for (unsigned i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = request.type; buffer.memory = request.memory; buffer.index = i;
        require_ioctl(fd_, VIDIOC_QUERYBUF, &buffer, "Query camera buffer");
        void* data = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
        if (data == MAP_FAILED) throw std::runtime_error("Map camera buffer failed");
        buffers_.push_back({data, buffer.length});
        require_ioctl(fd_, VIDIOC_QBUF, &buffer, "Queue camera buffer");
      }
      auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      require_ioctl(fd_, VIDIOC_STREAMON, &type, "Start camera"); started_ = true;
    } catch (...) { cleanup(); throw; }
  }
  ~Capture() { cleanup(); }
  Capture(const Capture&) = delete;
  Capture& operator=(const Capture&) = delete;
  bool fetch(cv::Mat& bgr, uint32_t& sequence, int64_t& received, bool& monotonic) {
    pollfd item{fd_, POLLIN, 0};
    int result;
    do { result = poll(&item, 1, 200); } while (result < 0 && errno == EINTR);
    if (result == 0) return false;
    if (result < 0 || (item.revents & (POLLERR | POLLHUP | POLLNVAL)))
      throw std::runtime_error("Camera poll/disconnect error");
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl_retry(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN) return false;
      throw std::runtime_error("Dequeue camera buffer: " + std::string(std::strerror(errno)));
    }
    if (buffer.index >= buffers_.size()) throw std::runtime_error("Invalid camera buffer index");
    bool valid = !(buffer.flags & V4L2_BUF_FLAG_ERROR) &&
                 buffer.bytesused >= size_t(stride_) * (height_ - 1) + width_ * 2 &&
                 buffer.bytesused <= buffers_[buffer.index].size;
    try {
      if (valid) {
        cv::Mat yuyv(height_, width_, CV_8UC2, buffers_[buffer.index].data, stride_);
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
        sequence = buffer.sequence;
        const auto host = luxi::mono();
        const int64_t stamp = int64_t(buffer.timestamp.tv_sec) * 1000000000 +
                              int64_t(buffer.timestamp.tv_usec) * 1000;
        monotonic = (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC &&
                    stamp > 0 && stamp <= host && host - stamp < 5000000000;
        received = monotonic ? stamp : host;
      }
    } catch (...) { ioctl_retry(fd_, VIDIOC_QBUF, &buffer); throw; }
    require_ioctl(fd_, VIDIOC_QBUF, &buffer, "Return camera buffer");
    return valid;
  }
};
}

class RealSenseCamera : public rclcpp::Node {
  std::string id_, serial_;
  int width_, height_, capture_width_, capture_height_, fps_, quality_;
  std::atomic<bool> active_{true};
  std::thread worker_;
  rclcpp::Publisher<luxi::Frame>::SharedPtr frames_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr images_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  void status(const std::string& phase, const std::string& reason, const luxi::Json& extra = {}) {
    auto j = luxi::Json{{"camera_id", id_}, {"serial", serial_}, {"backend", "realsense_v4l2"},
                       {"phase", phase}, {"reason", reason}, {"mock", false},
                       {"receive_mono_ns", std::to_string(luxi::mono())}};
    if (extra.is_object()) j.update(extra);
    std_msgs::msg::String m; m.data = j.dump(); status_->publish(m);
  }
  void capture() {
    cv::setNumThreads(1);
    uint64_t sequence = 0;
    while (active_ && rclcpp::ok()) {
      try {
        Capture camera(serial_, capture_width_, capture_height_, fps_);
        status("waiting", "等待 RealSense RGB 图像", {{"device", camera.device}});
        auto last_frame = luxi::mono(), report_start = last_frame;
        uint32_t previous = 0; bool seen = false;
        uint64_t interval_frames = 0, source_gaps = 0;
        while (active_ && rclcpp::ok()) {
          cv::Mat bgr; uint32_t source = 0; int64_t received = 0; bool monotonic = false;
          if (!camera.fetch(bgr, source, received, monotonic)) {
            if (luxi::mono() - last_frame > 8000000000) throw std::runtime_error("RGB frame timeout");
            continue;
          }
          if (seen && source == previous) continue;
          if (seen && source > previous + 1) source_gaps += source - previous - 1;
          previous = source; seen = true; last_frame = luxi::mono();
          if (bgr.cols != width_ || bgr.rows != height_)
            cv::resize(bgr, bgr, {width_, height_}, 0, 0, cv::INTER_AREA);
          luxi::Frame f;
          f.header.stamp = now(); f.header.frame_id = id_ + "_optical";
          f.camera_id = id_; f.sequence = ++sequence;
          f.receive_mono_ns = received;
          // V4L2 timestamps are host timestamps, not camera hardware timestamps.
          f.device_timestamp = 0;
          f.timestamp_quality = monotonic ? "v4l2_host_monotonic" : "host_receive_only";
          f.width = bgr.cols; f.height = bgr.rows; f.encoding = "jpeg";
          if (!cv::imencode(".jpg", bgr, f.data, {cv::IMWRITE_JPEG_QUALITY, quality_}))
            throw std::runtime_error("JPEG encoding failed");
          frames_->publish(f);
          sensor_msgs::msg::CompressedImage image;
          image.header = f.header; image.format = "jpeg"; image.data = f.data;
          images_->publish(image); ++interval_frames;
          if (last_frame - report_start >= 1000000000) {
            status("streaming", "RealSense RGB 图像采集中",
                   {{"device", camera.device}, {"fps", interval_frames * 1e9 / (last_frame - report_start)},
                    {"source_sequence", source}, {"source_frame_gaps", source_gaps},
                    {"capture_width", capture_width_}, {"capture_height", capture_height_},
                    {"width", width_}, {"height", height_}});
            report_start = last_frame; interval_frames = 0;
          }
        }
      } catch (const std::exception& e) {
        status("error", e.what());
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        for (int i = 0; i < 40 && active_ && rclcpp::ok(); ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }
 public:
  RealSenseCamera() : Node("realsense_camera") {
    id_ = declare_parameter<std::string>("camera_id", "head_rgb");
    serial_ = declare_parameter<std::string>("serial", "");
    width_ = declare_parameter("width", 640); height_ = declare_parameter("height", 360);
    capture_width_ = declare_parameter("capture_width", 1280);
    capture_height_ = declare_parameter("capture_height", 720);
    fps_ = declare_parameter("fps", 30); quality_ = declare_parameter("jpeg_quality", 90);
    if ((id_ != "head_rgb" && id_ != "left_wrist_rgb" && id_ != "right_wrist_rgb") ||
        serial_.empty() || serial_.find_first_not_of("0123456789") != std::string::npos ||
        width_ < 16 || width_ > 4096 || height_ < 16 || height_ > 4096 ||
        capture_width_ < 16 || capture_width_ > 4096 || capture_width_ % 2 ||
        capture_height_ < 16 || capture_height_ > 4096 || fps_ < 1 || fps_ > 60 || quality_ < 1 || quality_ > 100)
      throw std::invalid_argument("Invalid RealSense camera configuration");
    frames_ = create_publisher<luxi::Frame>("/luxi/cameras/" + id_, rclcpp::QoS(4).reliable());
    images_ = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/" + id_ + "/image/compressed", rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("/luxi/cameras/" + id_ + "/status", rclcpp::QoS(1).transient_local());
    worker_ = std::thread([this] { capture(); });
  }
  ~RealSenseCamera() { active_ = false; if (worker_.joinable()) worker_.join(); }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<RealSenseCamera>(); });
}
