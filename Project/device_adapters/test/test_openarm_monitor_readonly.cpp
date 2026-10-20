#include "../src/openarm_monitor.hpp"
#include <iostream>
using namespace luxi::monitor;
static void require(bool ok) { if (!ok) throw std::runtime_error("monitor parser check failed"); }
int main() {
  for (unsigned id = 1; id <= 8; ++id) {
    for (unsigned rid : {0, 21, 22, 23}) {
      auto f = request(id, rid);
      require(f.can_id == 0x7ff && f.can_dlc == 8 && f.data[0] == id && f.data[1] == 0);
      require(f.data[2] == (rid ? 0x33 : 0xcc) && f.data[3] == rid);
      require(f.data[4] == 0 && f.data[5] == 0 && f.data[6] == 0 && f.data[7] == 0);
    }
  }
  bool rejected = false;
  try { request(7, 10); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected);
  std::array<Motor, 8> motors{};
  canfd_frame f{};
  f.can_id = 0x15; f.len = 8;
  f.data[0] = 5; f.data[2] = 0x33;
  for (unsigned k = 0; k < 3; ++k) {
    float range[] = {12.5f, 10.f, 28.f};
    f.data[3] = 21 + k;
    std::memcpy(f.data + 4, &range[k], 4);
    consume(motors, f, 123);
    require(motors[4].seen == 0); // parameter replies are never position feedback
  }
  require(motors[4].calibrated());
  f.data[0] = 5; f.data[1] = 0x80; f.data[2] = 0;
  f.data[3] = 0xff; f.data[4] = 0xff; f.data[5] = 0xff;
  f.data[6] = 29; f.data[7] = 31;
  consume(motors, f, 456);
  require(motors[4].seen == 456 && motors[4].healthy());
  require(std::abs(motors[4].position()) < .001);
  require(motors[4].velocity() == 10 && motors[4].torque() == 28);
  f.data[0] = 6; consume(motors, f, 999); require(motors[4].seen == 456);
  f.data[0] = 0x85; consume(motors, f, 777); require(!motors[4].healthy());
  f.can_id |= CAN_RTR_FLAG; consume(motors, f, 888); require(motors[4].seen == 777);
  std::cout << "read-only monitor requests, per-motor scales, frame filtering and fault handling passed\n";
}
