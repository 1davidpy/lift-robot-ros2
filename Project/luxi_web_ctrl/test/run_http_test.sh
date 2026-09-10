#!/usr/bin/env bash
set -euo pipefail
ROOT=$1
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$ROOT/log/web-test/ros"
mkdir -p "$ROOT/log/web-test" "$ROOT/build/web-test/data"
"$ROOT/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl" --ros-args -p port:=18020 -p control_token:=web-test-token-123456 -p "directory:=$ROOT/build/web-test/data" > "$ROOT/log/web-test/server.log" 2>&1 &
server=$!
trap 'kill -INT "$server" 2>/dev/null || true; wait "$server" || true' EXIT
"$ROOT/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/http_probe"
