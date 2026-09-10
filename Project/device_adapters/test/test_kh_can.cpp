// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <luxi/kh_can.hpp>
using namespace luxi::kh;
TEST(KhCan, MappingDoesNotDependOnKernelNames) {
  Json cfg = {{"serial", "unit"}, {"channels", Json::array()}};
  Json devices = Json::array();
  for (int i = 0; i < 4; ++i) {
    cfg["channels"].push_back({{"channel", i},
                               {"name", "luxi" + std::to_string(i)},
                               {"bitrate", 1000000},
                               {"dbitrate", 5000000},
                               {"fd", true},
                               {"enabled", i < 2}});
    devices.push_back(
        {{"channel", i}, {"serial", "unit"}, {"interface", "can" + std::to_string(9 - i)}});
  }
  auto resolved = resolve(cfg, devices);
  EXPECT_EQ(resolved[0]["interface"], "can9");
  EXPECT_EQ(resolved[3]["interface"], "can6");
  auto bad = cfg;
  bad["serial"] = "another";
  EXPECT_THROW(resolve(bad, devices), std::runtime_error);
  bad = cfg;
  bad["channels"][1]["channel"] = 0;
  EXPECT_THROW(validate(bad), std::runtime_error);
  bad = cfg;
  bad["channels"][1]["name"] = "bad;name";
  EXPECT_THROW(validate(bad), std::runtime_error);
  devices.push_back(devices[0]);
  EXPECT_THROW(resolve(cfg, devices), std::runtime_error);
}
