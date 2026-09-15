#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
export ROS_LOG_DIR="$ROOT/log/ros"
case "${1:-}" in
 arms) exec ros2 run luxi_device_adapters openarm_adapter --ros-args -p "config:=$ROOT/Project/robot_bringup/config/arms-kh.json" -p mock:=false -p allow_hardware:=true ;;
 base) exec ros2 run luxi_device_adapters r680_adapter --ros-args -p mock:=false -p allow_hardware:=true -p interface:=luxi_base -p require_left_grip:=false ;;
 program)
  source "$ROOT/Project/luxi_web_ctrl/scripts/collection_env.sh"
  exec ros2 launch "$ROOT/Project/luxi_web_ctrl/scripts/program.launch.xml" \
   "directory:=$COLLECTION_DIRECTORY" "minimum_free_gib:=$COLLECTION_MIN_FREE_GIB" \
   "camera_config:=$COLLECTION_CAMERA_CONFIG" ;;
 *) exit 2;;
esac
