#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec 9>/tmp/luxi-handeye-record.lock
flock -n 9 || { echo 'A hand-eye recording is already running'; exit 1; }
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
export ROS_LOG_DIR="$ROOT/log/handeye"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$ROOT/tool/record_handeye.py" "$@"
