#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$ROOT/log/integration/ros"
RUN="$ROOT/build/test_runs/$(date +%Y%m%dT%H%M%S)-$$"
mkdir -p "$RUN" "$ROOT/log/integration"
setsid ros2 launch luxi_bringup teleop.launch.xml mock:=true base_enabled:=true "directory:=$RUN/data" minimum_free_gib:=0.01 quest_port:=15006 http_port:=18000 > "$ROOT/log/integration/launch-$$.log" 2>&1 &
launcher=$!
trap 'kill -TERM -- -"$launcher" 2>/dev/null || true; wait "$launcher" 2>/dev/null || true' EXIT
"$ROOT/build/install/luxi_bringup/lib/luxi_bringup/integration_probe" "$ROOT" "$RUN/data"
"$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_recording_check" "$RUN/data"
