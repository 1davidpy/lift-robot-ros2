// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <cstdlib>
#include <luxi/hikrobot.hpp>
using namespace luxi::hik;
extern "C" {
int32_t MV3D_RGBD_Initialize() { return 0; }
int32_t MV3D_RGBD_Release() { return 0; }
int32_t MV3D_RGBD_OpenDeviceBySerialNumber(void** h, const char*) {
  *h = new uint32_t(0);
  return 0;
}
int32_t MV3D_RGBD_CloseDevice(void** h) {
  delete static_cast<uint32_t*>(*h);
  *h = nullptr;
  return 0;
}
int32_t MV3D_RGBD_Start(void*) { return 0; }
int32_t MV3D_RGBD_Stop(void*) { return 0; }
int32_t MV3D_RGBD_SetParam(void*, const char*, Param* p) { return p->type == 4 ? 0 : -1; }
int32_t MV3D_RGBD_FetchFrame(void* h, Frame* f, uint32_t) {
  static uint8_t pixels[12] = {255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0};
  f->count = std::getenv("LUXI_TEST_BAD_FRAME") ? 11 : 1;
  auto& im = f->images[0];
  im = {};
  im.type = 0x02180021;
  im.width = 2;
  im.height = 2;
  im.data = pixels;
  im.length = 12;
  im.sequence = ++*static_cast<uint32_t*>(h);
  im.timestamp = 123456 + im.sequence;
  return 0;
}
}
