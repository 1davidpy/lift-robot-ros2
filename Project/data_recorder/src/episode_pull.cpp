// SPDX-License-Identifier: Apache-2.0
#include <luxi/transfer.hpp>
#include <iostream>
int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::invalid_argument("usage: episode_pull http://board:8000 episode-id destination");
    luxi::receive_episode(argv[1], argv[2], argv[3]);
    std::cout << "Verified episode: " << luxi::fs::path(argv[3]) / argv[2] << '\n';
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
