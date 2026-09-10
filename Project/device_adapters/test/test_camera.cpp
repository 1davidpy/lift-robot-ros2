// SPDX-License-Identifier: Apache-2.0
// Quest/OpenArm/Hikrobot behavior ported from Enactic openarm_teleop; see
// docs/CPP迁移与接口说明.md.
#include <gtest/gtest.h>

#include <luxi/hikrobot.hpp>
TEST(Camera, SDKAbiPixelsAndTimestamps) {
  luxi::hik::Camera c(FAKE_SDK, "test");
  cv::Mat image;
  uint32_t seq;
  int64_t t;
  ASSERT_TRUE(c.fetch(image, seq, t));
  EXPECT_EQ(seq, 1u);
  EXPECT_EQ(t, 123457);
  EXPECT_EQ(image.rows, 2);
  EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));
  ASSERT_TRUE(c.fetch(image, seq, t));
  EXPECT_EQ(seq, 2u);
}
TEST(Camera, InvalidLibrary) {
  EXPECT_THROW(luxi::hik::Camera("/missing/sdk.so", "serial"), std::runtime_error);
}
TEST(Camera, InvalidCount) {
  luxi::hik::Camera c(FAKE_SDK, "test");
  setenv("LUXI_TEST_BAD_FRAME", "1", 1);
  cv::Mat im;
  uint32_t s;
  int64_t t;
  EXPECT_THROW(c.fetch(im, s, t), std::runtime_error);
  unsetenv("LUXI_TEST_BAD_FRAME");
}
