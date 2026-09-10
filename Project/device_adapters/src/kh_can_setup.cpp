// SPDX-License-Identifier: Apache-2.0
#include <net/if.h>
#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <luxi/kh_can.hpp>
#include <vector>
using luxi::kh::Json;
static void ip(std::vector<std::string> args) {
  args.insert(args.begin(), "/usr/sbin/ip");
  std::vector<char*> argv;
  for (auto& a : args) argv.push_back(a.data());
  argv.push_back(nullptr);
  pid_t pid = fork();
  if (pid < 0) throw std::runtime_error("fork ip");
  if (!pid) {
    execv(argv[0], argv.data());
    _exit(127);
  }
  int status = 0;
  if (waitpid(pid, &status, 0) != pid || !WIFEXITED(status) || WEXITSTATUS(status))
    throw std::runtime_error("ip configuration failed; inspect interfaces before retrying");
}
int main(int argc, char** argv) {
  try {
    auto devices = luxi::kh::inventory();
    if (argc == 1) {
      std::cout << devices.dump(2) << '\n';
      return devices.empty() ? 2 : 0;
    }
    if (argc > 3)
      throw std::runtime_error("usage: kh_can_setup [config.json [--listen|--apply|--down]]");
    std::ifstream in(argv[1]);
    Json cfg;
    in >> cfg;
    auto plan = luxi::kh::resolve(cfg, devices);
    std::cout << plan.dump(2) << '\n';
    if (argc == 2) return 0;
    std::string mode = argv[2];
    if (mode != "--listen" && mode != "--apply" && mode != "--down")
      throw std::runtime_error("unknown mode");
    if (geteuid()) throw std::runtime_error("configuration requires root");
    // Validate all names before making any change. Do not rename another device.
    for (const auto& c : plan) {
      std::string old = c.at("interface"), name = c.at("name");
      if (old != name && if_nametoindex(name.c_str()))
        throw std::runtime_error("target name already exists: " + name);
    }
    for (const auto& c : plan) {
      std::string old = c.at("interface"), name = c.at("name");
      ip({"link", "set", "dev", old, "down"});
      if (mode == "--down") continue;
      if (old != name) ip({"link", "set", "dev", old, "name", name});
      if (!c.at("enabled").get<bool>()) continue;
      std::vector<std::string> args{
          "link", "set", "dev",     name,
          "type", "can", "bitrate", std::to_string(c.at("bitrate").get<int>())};
      if (c.at("fd").get<bool>()) {
        args.insert(args.end(),
                    {"dbitrate", std::to_string(c.at("dbitrate").get<int>()), "fd", "on"});
      } else
        args.insert(args.end(), {"fd", "off"});
      args.insert(args.end(), {"listen-only", mode == "--listen" ? "on" : "off", "restart-ms", "0",
                               "berr-reporting", "on"});
      ip(args);
      ip({"link", "set", "dev", name, "txqueuelen", "256"});
      ip({"link", "set", "dev", name, "up"});
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
