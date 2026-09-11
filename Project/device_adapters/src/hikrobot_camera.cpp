// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <luxi/core.hpp>
#include <luxi/hikrobot.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
class CameraNode : public rclcpp::Node {
  std::unique_ptr<luxi::hik::Camera> camera_;
  std::string id_, library_, serial_;
  bool mock_ = true;
  int working_ = 2, mode_ = 8;
  int64_t retry_after_ = 0, last_frame_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  void status(const std::string& phase, const std::string& reason) {
    std_msgs::msg::String m;
    m.data = luxi::Json{{"camera_id", id_}, {"phase", phase}, {"reason", reason},
      {"serial", serial_}, {"mock", mock_}, {"receive_mono_ns", std::to_string(luxi::mono())}}.dump();
    status_->publish(m);
  }
  int width_, height_, quality_;
  uint32_t sequence_ = 0, last_sequence_ = 0;
  bool seen_ = false;
  rclcpp::Publisher<luxi::Frame>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  CameraNode() : Node("hikrobot_camera") {
    id_ = declare_parameter("camera_id", "head_rgb");
    bool mock = declare_parameter("mock", true);
    width_ = declare_parameter("width", 640);
    height_ = declare_parameter("height", 360);
    quality_ = declare_parameter("jpeg_quality", 90);
    if (width_ < 16 || width_ > 4096 || height_ < 16 || height_ > 4096 || quality_ < 1 ||
        quality_ > 100)
      throw std::invalid_argument("camera output parameters");
    mock_ = mock;
    library_ = declare_parameter<std::string>("sdk_library", "");
    serial_ = declare_parameter<std::string>("serial", "");
    working_ = declare_parameter("working_mode", 2);
    mode_ = declare_parameter("image_mode", 8);
    if (id_ != "head_rgb" && id_ != "left_wrist_rgb" && id_ != "right_wrist_rgb")
      throw std::invalid_argument("unknown camera_id");
    status_ = create_publisher<std_msgs::msg::String>("/luxi/cameras/" + id_ + "/status", rclcpp::QoS(1).transient_local());
    pub_ = create_publisher<luxi::Frame>("/luxi/cameras/" + id_, rclcpp::QoS(4).reliable());
    image_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/" + id_ + "/image/compressed", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] { capture(); });
  }
  void capture() {
    try {
      if (!mock_ && !camera_) {
        if (luxi::mono() < retry_after_) return;
        retry_after_ = luxi::mono() + 3000000000;
        if (library_.empty()) throw std::runtime_error("缺少 MV3D RGB-D SDK 路径，请配置 sdk_library");
        camera_ = std::make_unique<luxi::hik::Camera>(library_, serial_, working_, mode_);
        seen_ = false;
        last_frame_ = luxi::mono();
        status("waiting", "相机已打开，等待 RGB 图像");
      }
      cv::Mat bgr;
      int64_t device = 0;
      uint32_t seq;
      if (camera_) {
        if (!camera_->fetch(bgr, seq, device)) {
          if (luxi::mono() - last_frame_ > 3000000000)
            throw std::runtime_error("RGB 取帧超时，检查 USB 连接和相机模式");
          return;
        }
      } else {
        seq = ++sequence_;
        bgr = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(seq % 255, 80, 130));
        cv::putText(bgr, id_ + " " + std::to_string(seq), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, .7,
                    {255, 255, 255}, 2);
      }
      if (seen_ && seq == last_sequence_) {
        if (luxi::mono() - last_frame_ > 3000000000)
          throw std::runtime_error("RGB 帧序号停止更新");
        return;
      }
      last_frame_ = luxi::mono();
      last_sequence_ = seq;
      seen_ = true;
      luxi::Frame f;
      f.receive_mono_ns = luxi::mono();
      f.header.stamp = now();
      f.header.frame_id = id_ + "_optical";
      f.camera_id = id_;
      f.sequence = seq;
      f.device_timestamp = device;
      f.timestamp_quality = camera_ ? "host_receive_device_unmapped" : "mock_host";
      if (bgr.cols != width_ || bgr.rows != height_)
        cv::resize(bgr, bgr, {width_, height_}, 0, 0, cv::INTER_AREA);
      f.width = bgr.cols;
      f.height = bgr.rows;
      f.encoding = "jpeg";
      if (!cv::imencode(".jpg", bgr, f.data, {cv::IMWRITE_JPEG_QUALITY, quality_}))
        throw std::runtime_error("jpeg encode");
      pub_->publish(f);
      sensor_msgs::msg::CompressedImage im;
      im.header = f.header;
      im.format = "jpeg";
      im.data = f.data;
      image_->publish(im);
      if (seq % 15 == 0) status("streaming", "RGB 图像采集中");
    } catch (const std::exception& e) {
      camera_.reset();
      retry_after_ = luxi::mono() + 3000000000;
      status("error", e.what());
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "camera: %s", e.what());
    }
  }
};
int main(int argc, char** argv) {
  return luxi::node_main(argc, argv, [] { return std::make_shared<CameraNode>(); });
}
