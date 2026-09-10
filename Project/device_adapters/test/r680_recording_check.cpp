// SPDX-License-Identifier: Apache-2.0
#include <httplib.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("episode directory required");
    int count = 0;
    for (const auto& e : std::filesystem::directory_iterator(argv[1])) {
      if (!std::filesystem::exists(e.path() / "manifest.json")) continue;
      std::ifstream in(e.path() / "manifest.json");
      nlohmann::json m;
      in >> m;
      for (const auto& topic :
           {"/luxi/base/state", "/luxi/base/applied", "/luxi/base/can_rx", "/luxi/base/status"})
        if (m.at("counts").value(topic, 0) < 1)
          throw std::runtime_error(std::string("missing recorded topic ") + topic);
      ++count;
    }
    if (count != 1) throw std::runtime_error("expected one committed episode");
    httplib::Client client("127.0.0.1", 18000);
    auto response = client.Get("/api/state");
    if (!response || response->status != 200) throw std::runtime_error("HTTP state");
    auto state = nlohmann::json::parse(response->body);
    if (state.at("base").at("schema") != "r680_v1" || !state.at("base").at("valid").get<bool>() ||
        !state.at("base").at("mock").get<bool>())
      throw std::runtime_error("LAN base feedback");
    auto stopped = client.Post("/api/base", "{\"command\":\"stop\"}", "application/json");
    if (!stopped || stopped->status != 200 ||
        !nlohmann::json::parse(stopped->body).at("success").get<bool>())
      throw std::runtime_error("HTTP base stop service");
    auto rejected = client.Post("/api/base", "{\"command\":\"invalid\"}", "application/json");
    if (!rejected || rejected->status != 409)
      throw std::runtime_error("HTTP rejection propagation");
    std::cout << "PASS: R680 raw/feedback/applied/status recorded in episode, LAN state and "
                 "control service\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
