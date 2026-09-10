// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <luxi/r680.hpp>
using namespace luxi::r680;
static std::array<can_frame, 3> example() {
  const uint8_t bytes[24] = {0x7b, 0, 0, 0xc8, 0xff, 0x9c, 1, 0x2c, 0,    0,    0,    0,
                             0x40, 0, 0, 0,    0,    0,    0, 0,    0x5d, 0xc0, 0x20, 0x7d};
  std::array<can_frame, 3> fs{};
  for (int i = 0; i < 3; ++i) {
    fs[i].can_id = 0x101 + i;
    fs[i].can_dlc = 8;
    std::copy_n(bytes + 8 * i, 8, fs[i].data);
  }
  return fs;
}
TEST(R680Codec, CommandGoldenVector) {
  auto f = encode({{.123, -.456, .789}});
  const std::array<uint8_t, 8> expected{{0, 0x7b, 0xfe, 0x38, 3, 0x15, 0, 0}};
  EXPECT_EQ(f.can_id, 0x181u);
  EXPECT_EQ(f.can_dlc, 8);
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), f.data));
  EXPECT_THROW(encode({{NAN, 0, 0}}), std::invalid_argument);
  EXPECT_THROW(encode({{33, 0, 0}}), std::invalid_argument);
}
TEST(R680Codec, FeedbackGoldenVector) {
  auto f = example();
  Decoder d;
  EXPECT_FALSE(d.consume(f[0], 1000000000));
  EXPECT_FALSE(d.consume(f[1], 1001000000));
  auto s = d.consume(f[2], 1002000000);
  ASSERT_TRUE(s);
  EXPECT_DOUBLE_EQ(s->velocity[0], .2);
  EXPECT_DOUBLE_EQ(s->velocity[1], -.1);
  EXPECT_DOUBLE_EQ(s->velocity[2], .3);
  EXPECT_NEAR(s->acceleration[2], 9.8, .01);
  EXPECT_DOUBLE_EQ(s->voltage, 24);
  EXPECT_EQ(s->mono_ns[1], 1001000000);
  EXPECT_EQ(d.accepted, 1u);
}
TEST(R680Codec, RejectMalformedGroups) {
  auto f = example();
  for (int mode = 0; mode < 6; ++mode) {
    Decoder d;
    auto frames = f;
    if (mode == 0) frames[0].can_id |= CAN_EFF_FLAG;
    if (mode == 1) frames[1].can_dlc = 7;
    if (mode == 2) frames[2].data[6] ^= 1;
    if (mode == 3) frames[2].data[7] = 0;
    d.consume(frames[0], 1000000000);
    d.consume(frames[1], 1001000000);
    if (mode == 4) d.consume(frames[1], 1002000000);
    EXPECT_FALSE(d.consume(frames[2], mode == 5 ? 1021000000 : 1003000000));
    EXPECT_GT(d.rejected, 0u);
  }
}
TEST(R680Codec, NewStartAndClockRollback) {
  auto f = example();
  Decoder d;
  d.consume(f[0], 1000000000);
  d.consume(f[1], 1001000000);
  d.consume(f[0], 1050000000);
  EXPECT_FALSE(d.consume(f[2], 1051000000));
  d.consume(f[0], 1100000000);
  EXPECT_FALSE(d.consume(f[1], 1099000000));
  EXPECT_FALSE(d.consume(f[2], 1101000000));
}
TEST(R680Mapping, PS2DirectionsAndReverseSteering) {
  Mapping m;
  EXPECT_GT(m.map({{0, 1, 0, 0}})[0], 0);
  EXPECT_LT(m.map({{1, 0, 0, 0}})[1], 0);
  EXPECT_LT(m.map({{0, 0, 1, 0}})[2], 0);
  EXPECT_GT(m.map({{0, -1, 1, 0}})[2], 0);
  m.reverse_steering = false;
  EXPECT_LT(m.map({{0, -1, 1, 0}})[2], 0);
  EXPECT_EQ(m.map({{.1, -.1, .1, 1}}), Velocity{});
}
static Gate ready() {
  Gate g;
  g.quest_time = g.feedback_time = 1000000000;
  g.tracking = true;
  g.stop = 0;
  return g;
}
TEST(R680Gate, NeutralEnableAndDeadman) {
  auto g = ready();
  g.sticks[1] = 1;
  EXPECT_FALSE(g.start(1000000000));
  g.sticks = {};
  g.grip = 1;
  EXPECT_FALSE(g.start(1000000000));
  g.grip = 0;
  ASSERT_TRUE(g.start(1000000000));
  g.sticks[1] = 1;
  EXPECT_EQ(g.command(1000000000), Velocity{});
  g.grip = 1;
  EXPECT_GT(g.command(1000000000)[0], 0);
  g.grip = 0;
  EXPECT_EQ(g.command(1000000000), Velocity{});
}
TEST(R680Gate, TimeoutAndReconnectCannotResume) {
  auto g = ready();
  ASSERT_TRUE(g.start(1000000000));
  g.grip = 1;
  g.sticks[1] = 1;
  EXPECT_EQ(g.command(1300000000), Velocity{});
  EXPECT_FALSE(g.enabled);
  g.quest_time = g.feedback_time = 1300000000;
  EXPECT_EQ(g.command(1300000000), Velocity{});
}
TEST(R680Gate, FeedbackStopMenuTracking) {
  for (int mode = 0; mode < 4; ++mode) {
    auto g = ready();
    ASSERT_TRUE(g.start(1000000000));
    g.grip = 1;
    g.sticks[1] = 1;
    if (mode == 0) g.stop = 1;
    if (mode == 1) g.menu = true;
    if (mode == 2) g.tracking = false;
    if (mode == 3) {
      g.feedback_time = 1;
      g.quest_time = 1000000000;
    }
    EXPECT_EQ(g.command(1000000000), Velocity{});
    EXPECT_FALSE(g.enabled);
  }
}

TEST(R680Gate, NonfiniteInputCannotEnableOrMove) {
  auto g = ready();
  g.grip = NAN;
  EXPECT_FALSE(g.start(1000000000));
  g.grip = 0;
  g.sticks[0] = NAN;
  EXPECT_FALSE(g.start(1000000000));
  g.sticks = {};
  ASSERT_TRUE(g.start(1000000000));
  g.grip = NAN;
  EXPECT_EQ(g.command(1000000000), Velocity{});
  EXPECT_FALSE(g.enabled);
}
