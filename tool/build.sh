#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
set -u
export MAKEFLAGS="-j2 -l4"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export ROS_LOG_DIR="$ROOT/log/ros"
if [[ -d "$ROOT/build/depth-sdk/install" ]]; then
 export CMAKE_PREFIX_PATH="$ROOT/build/depth-sdk/install:${CMAKE_PREFIX_PATH:-}"
fi
cd "$ROOT"
colcon --log-base "$ROOT/log/colcon" build --base-paths "$ROOT/Project" --build-base "$ROOT/build/packages" --install-base "$ROOT/build/install" --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG' -DLUXI_ROOT="$ROOT" "$@"
