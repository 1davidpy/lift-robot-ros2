// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
namespace luxi::kh {
namespace fs = std::filesystem;
using Json = nlohmann::json;
inline std::string read(const fs::path& path) {
  std::ifstream in(path);
  std::string s;
  std::getline(in, s);
  return s;
}
// kcan 1.4.2 exposes controller index and corresponding SocketCAN name here.
inline Json inventory(const fs::path& sys = "/sys") {
  Json result = Json::array();
  if (!fs::exists(sys / "class/kcan")) return result;
  for (const auto& e : fs::directory_iterator(sys / "class/kcan")) {
    auto path = fs::canonical(e.path());
    auto usb = path;
    while (usb != usb.root_path() && !fs::exists(usb / "idVendor")) usb = usb.parent_path();
    if (read(usb / "idVendor") != "395e" || read(usb / "idProduct") != "0020") continue;
    const auto index = read(path / "ctrlr_number");
    if (index.empty()) throw std::runtime_error("missing kcan controller index");
    result.push_back({{"interface", read(path / "ndev")},
                      {"channel", std::stoi(index)},
                      {"serial", read(usb / "serial")},
                      {"usb_path", usb.string()},
                      {"driver", "kcan"}});
  }
  return result;
}
inline void validate(const Json& cfg) {
  if (cfg.at("serial").get<std::string>().empty()) throw std::runtime_error("USB serial required");
  std::set<int> channels;
  std::set<std::string> names;
  if (cfg.at("channels").size() != 4) throw std::runtime_error("four channels required");
  for (const auto& c : cfg.at("channels")) {
    int ch = c.at("channel"), bitrate = c.at("bitrate"), dbitrate = c.at("dbitrate");
    auto name = c.at("name").get<std::string>();
    if (ch < 0 || ch > 3 || !channels.insert(ch).second || !names.insert(name).second ||
        name.empty() || name.size() >= 16 ||
        name.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
            std::string::npos ||
        bitrate <= 0 || bitrate > 1000000 || dbitrate < bitrate || dbitrate > 8000000)
      throw std::runtime_error("invalid channel/name/bitrate");
    c.at("enabled").get<bool>();
    c.at("fd").get<bool>();
  }
}
inline Json resolve(const Json& cfg, const Json& devices) {
  validate(cfg);
  Json result = Json::array();
  std::set<std::string> interfaces;
  for (const auto& c : cfg.at("channels")) {
    Json matches = Json::array();
    for (const auto& d : devices)
      if (d.at("serial") == cfg.at("serial") && d.at("channel") == c.at("channel"))
        matches.push_back(d);
    if (matches.size() != 1) throw std::runtime_error("missing or ambiguous USB channel");
    auto item = c;
    auto iface = matches[0].at("interface").get<std::string>();
    if (iface.empty() || iface.size() >= 16 || !interfaces.insert(iface).second)
      throw std::runtime_error("invalid resolved interface");
    item["interface"] = iface;
    result.push_back(item);
  }
  return result;
}
}  // namespace luxi::kh
