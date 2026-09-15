#!/usr/bin/env bash
# Standalone receiver: no ROS or camera SDK required on the receiving PC.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=${1:-"$ROOT/build/receiver"}
mkdir -p "$OUT"
# Ubuntu/Debian dependencies: g++ pkg-config libcpp-httplib-dev libssl-dev nlohmann-json3-dev
# Linux and macOS (with these libraries); Windows can use WSL with a destination under /mnt/.
read -ra FLAGS <<< "$(pkg-config --cflags --libs cpp-httplib openssl)"
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -pthread \
 -I"$ROOT/Project/data_recorder/include" \
 "$ROOT/Project/data_recorder/src/episode_receive.cpp" "${FLAGS[@]}" -o "$OUT/episode_receive"
printf 'Receiver built: %s/episode_receive\n' "$OUT"
