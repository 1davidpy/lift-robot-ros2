#!/usr/bin/env bash
# Network receiver only: no CAN configuration and no motor control.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
exec "$ROOT/build/install/luxi_quest_receiver/lib/luxi_quest_receiver/quest_receiver" \
  --ros-args -p host:=0.0.0.0 -p port:=5006
