#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$ROOT/log/r680/ros"
mkdir -p "$ROOT/log/r680"
setsid "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_adapter" --ros-args -p mock:=true > "$ROOT/log/r680/adapter-$$.log" 2>&1 &
adapter=$!
setsid "$ROOT/build/install/luxi_quest_receiver/lib/luxi_quest_receiver/quest_receiver" --ros-args -p port:=15007 > "$ROOT/log/r680/quest-$$.log" 2>&1 &
receiver=$!
trap 'kill -TERM -- -"$adapter" -"$receiver" 2>/dev/null || true; wait "$adapter" "$receiver" 2>/dev/null || true' EXIT
"$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_integration_probe"
