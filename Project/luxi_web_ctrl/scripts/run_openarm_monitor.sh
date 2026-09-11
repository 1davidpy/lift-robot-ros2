#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
exec "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter" --ros-args \
 -p "config:=$ROOT/Project/robot_bringup/config/arms.json" \
 -p mock:=false -p allow_hardware:=true -p monitor_only:=true
