// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#pragma once
// ABI carried over from the working Python ctypes wrapper. Vendor headers/ARM64
// binary must be verified as a matching MV3D_RGBD SDK before physical deployment.
#include <dlfcn.h>

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
namespace luxi::hik {
struct Image {
  int64_t type;
  uint32_t width, height;
  uint8_t* data;
  uint32_t length, sequence;
  int64_t timestamp;
  uint8_t reserved[16];
};
struct Frame {
  uint32_t count;
  Image images[10];
  uint8_t reserved[16];
};
struct Enum {
  uint32_t current, count, supported[16];
};
struct Int {
  int64_t current, max, min, increment;
};
union Value {
  Enum enumeration;
  Int integer;
  float floating[3];
  char string[256];
  int32_t boolean;
};
struct Param {
  int32_t type;
  Value value;
};
class Camera {
  void *lib_ = nullptr, *handle_ = nullptr;
  bool initialized_ = false, started_ = false;
  int32_t (*initialize_)() = nullptr;
  int32_t (*release_)() = nullptr;
  int32_t (*open_)(void**, const char*) = nullptr;
  int32_t (*close_)(void**) = nullptr;
  int32_t (*start_)(void*) = nullptr;
  int32_t (*stop_)(void*) = nullptr;
  int32_t (*fetch_)(void*, Frame*, uint32_t) = nullptr;
  int32_t (*set_)(void*, const char*, Param*) = nullptr;
  template <class T>
  T symbol(const char* name) {
    auto s = dlsym(lib_, name);
    if (!s) throw std::runtime_error(std::string("Missing MV3D symbol ") + name);
    return reinterpret_cast<T>(s);
  }
  static void check(int32_t code) {
    if (code != 0) throw std::runtime_error("MV3D status=" + std::to_string(code));
  }
  void cleanup() {
    if (started_) stop_(handle_);
    if (handle_ && close_) close_(&handle_);
    if (initialized_) release_();
    if (lib_) dlclose(lib_);
    lib_ = nullptr;
  }

 public:
  Camera(const std::string& library, const std::string& serial, int working = 2, int mode = 8) {
    if (serial.empty()) throw std::invalid_argument("camera serial required");
    lib_ = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!lib_) throw std::runtime_error(dlerror());
    try {
      initialize_ = symbol<decltype(initialize_)>("MV3D_RGBD_Initialize");
      release_ = symbol<decltype(release_)>("MV3D_RGBD_Release");
      open_ = symbol<decltype(open_)>("MV3D_RGBD_OpenDeviceBySerialNumber");
      close_ = symbol<decltype(close_)>("MV3D_RGBD_CloseDevice");
      start_ = symbol<decltype(start_)>("MV3D_RGBD_Start");
      stop_ = symbol<decltype(stop_)>("MV3D_RGBD_Stop");
      fetch_ = symbol<decltype(fetch_)>("MV3D_RGBD_FetchFrame");
      set_ = symbol<decltype(set_)>("MV3D_RGBD_SetParam");
      check(initialize_());
      initialized_ = true;
      check(open_(&handle_, serial.c_str()));
      Param p{};
      p.type = 4;
      p.value.enumeration.current = working;
      check(set_(handle_, "CameraWorkingMode", &p));
      p.value.enumeration.current = mode;
      check(set_(handle_, "ImageMode", &p));
      check(start_(handle_));
      started_ = true;
    } catch (...) {
      cleanup();
      throw;
    }
  }
  ~Camera() { cleanup(); }
  bool fetch(cv::Mat& bgr, uint32_t& seq, int64_t& timestamp) {
    Frame f{};
    if (fetch_(handle_, &f, 100) != 0) return false;
    if (f.count > 10) throw std::runtime_error("SDK frame count");
    for (uint32_t n = 0; n < f.count; n++) {
      auto& im = f.images[n];
      if (!im.data || im.width == 0 || im.height == 0 || im.width > 8192 || im.height > 8192)
        continue;
      size_t pixels = size_t(im.width) * im.height;
      cv::Mat data;
      if (im.type == 0x80180001) {
        if (im.length == 0 || im.length > 64000000) continue;
        bgr = cv::imdecode(cv::Mat(1, im.length, CV_8UC1, im.data), cv::IMREAD_COLOR);
      } else if (im.type == 0x02100032 && im.length >= pixels * 2) {
        cv::cvtColor(cv::Mat(im.height, im.width, CV_8UC2, im.data), bgr, cv::COLOR_YUV2BGR_YUY2);
      } else if ((im.type == 0x020c8001 || im.type == 0x020c8002) && im.length >= pixels * 3 / 2) {
        cv::cvtColor(cv::Mat(im.height * 3 / 2, im.width, CV_8UC1, im.data), bgr,
                     im.type == 0x020c8001 ? cv::COLOR_YUV2BGR_NV12 : cv::COLOR_YUV2BGR_NV21);
      } else if (im.type == 0x02180021 && im.length >= pixels * 3) {
        std::vector<cv::Mat> planes{cv::Mat(im.height, im.width, CV_8UC1, im.data + 2 * pixels),
                                    cv::Mat(im.height, im.width, CV_8UC1, im.data + pixels),
                                    cv::Mat(im.height, im.width, CV_8UC1, im.data)};
        cv::merge(planes, bgr);
      } else
        continue;
      if (bgr.empty()) continue;
      seq = im.sequence;
      timestamp = im.timestamp;
      return true;
    }
    return false;
  }
};
}  // namespace luxi::hik
