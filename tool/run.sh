#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
cd "$ROOT"
exec ros2 launch luxi_bringup teleop.launch.xml "directory:=$ROOT/data" "$@"
