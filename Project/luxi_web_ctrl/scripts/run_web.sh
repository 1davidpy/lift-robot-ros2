#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
source "$ROOT/Project/luxi_web_ctrl/scripts/collection_env.sh"
export ROS_LOG_DIR="$ROOT/log/ros"
exec "$ROOT/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl" --ros-args -p host:=0.0.0.0 \
 -p "directory:=$COLLECTION_DIRECTORY" -p "ram_staging:=$COLLECTION_RAM" \
 -p mock:=false -p base_enabled:=true -p no_cameras:=false
