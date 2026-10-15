// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <luxi/lift_uart.hpp>
#include <luxi/coordinated_control.hpp>

namespace lp = luxi::lift;
TEST(LiftUart, PublishedWireExamples) {
  const auto move = lp::request(1, lp::Move, 1, -50000, true);
  EXPECT_EQ(lp::uart_frame(1, move.data, 8), (std::vector<uint8_t>{
    0xa5,0x5a,0x09,0x01,0x10,0x01,0xb0,0x3c,0xff,0xff,0x00,0x01,0x49,0x42}));
  const auto stop = lp::request(1, lp::Stop, 2, 0, true);
  EXPECT_EQ(lp::uart_frame(1, stop.data, 8), (std::vector<uint8_t>{
    0xa5,0x5a,0x09,0x01,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0xa3,0x97}));
}
TEST(LiftUart, SplitNoiseAndCorruptFrames) {
  lp::UartParser parser;
  const uint8_t ack[]{4,1,0,0x33,31,0,0,0};
  const auto frame = lp::uart_frame(0x81, ack, 8);
  auto corrupt = frame; corrupt[6] ^= 1;
  const uint8_t noise[]{0,0xa5,0,0x5a};
  parser.feed(noise, sizeof(noise), 100, 1000);
  parser.feed(corrupt.data(), corrupt.size(), 101, 1001);
  parser.feed(frame.data(), 2, 102, 1002);
  lp::UartPacket packet;
  EXPECT_FALSE(parser.next(packet));
  parser.feed(frame.data() + 2, frame.size() - 2, 103, 1003);
  ASSERT_TRUE(parser.next(packet));
  EXPECT_EQ(packet.type, 0x81); EXPECT_EQ(packet.data[4], 31);
  EXPECT_EQ(packet.first_ns, 102); EXPECT_EQ(parser.errors, 1u);
  EXPECT_FALSE(parser.next(packet));
}
TEST(LiftUart, TransportSpecificRangeAndGuard) {
  EXPECT_NO_THROW(lp::request(1, lp::Move, 0, -180000, true));
  EXPECT_THROW(lp::request(1, lp::Move, 0, -180001, true), std::invalid_argument);
  EXPECT_THROW(lp::request(1, lp::Move, 0, 180001, true), std::invalid_argument);
  EXPECT_THROW(lp::request(1, lp::Info, 0, 25), std::invalid_argument);
  EXPECT_NO_THROW(lp::request(1, lp::Info, 0, 30, true));
  lp::Sample s; s.first_ns = s.received_ns = 1000000000; s.flags = 0x33; s.feedback_age_ms = 100; s.guard = 4;
  EXPECT_FALSE(s.healthy(1100000000)); // CAN bus-off.
  s.uart = true;
  EXPECT_TRUE(s.healthy(1100000000)); // UART historical RX overflow, not CAN bus-off.
  s.flags &= ~1;
  EXPECT_FALSE(s.healthy(1100000000));
}
TEST(LiftUart, LockedPreparationHasStrictBounds) {
  lp::Sample s; s.uart = true; s.first_ns = s.received_ns = 2000000000;
  s.flags = 0x73; s.feedback_age_ms = 800; s.current_um = 1000;
  EXPECT_TRUE(s.startup_healthy(2100000000, 1000000000, 1000));
  s.flags |= 4; EXPECT_FALSE(s.startup_healthy(2100000000, 1000000000, 1000)); s.flags &= ~4;
  s.current_um = 1011; EXPECT_FALSE(s.startup_healthy(2100000000, 1000000000, 1000)); s.current_um = 1000;
  s.speed_rpm = 3; EXPECT_FALSE(s.startup_healthy(2100000000, 1000000000, 1000)); s.speed_rpm = 0;
  s.fault = 1; EXPECT_FALSE(s.startup_healthy(2100000000, 1000000000, 1000)); s.fault = 0;
  EXPECT_FALSE(s.startup_healthy(3100000000, 1000000000, 1000));
}
TEST(LiftUart, CoordinatorDoesNotTrustUnboundedGrace) {
  luxi::Json j{{"transport","uart"}, {"feedback_healthy",true}, {"feedback_first_mono_ns","2000000000"},
    {"feedback_age_ms",800}, {"startup_feedback_grace",true}, {"motion_started_mono_ns","1000000000"},
    {"phase","move_ack"}, {"flags",0x73}, {"fault",0}, {"speed_rpm",0},
    {"current_um",0}, {"startup_origin_um",0}};
  EXPECT_TRUE(luxi::coordinated_lift_feedback(j, 2100000000));
  j["phase"] = "stopping"; // Cancel keeps only the original locked startup deadline.
  EXPECT_TRUE(luxi::coordinated_lift_feedback(j, 2100000000));
  j["flags"] = 0x77; EXPECT_FALSE(luxi::coordinated_lift_feedback(j, 2100000000));
  j["flags"] = 0x73; j["motion_started_mono_ns"] = "1";
  EXPECT_FALSE(luxi::coordinated_lift_feedback(j, 2100000000));
  j["motion_started_mono_ns"] = "1000000000"; j["transport"] = "can";
  EXPECT_FALSE(luxi::coordinated_lift_feedback(j, 2100000000));
}
