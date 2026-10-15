#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
export ROS_LOG_DIR="$ROOT/log/ros"
case "${1:-}" in
 arms) exec ros2 run luxi_device_adapters openarm_adapter --ros-args -p "config:=$ROOT/Project/robot_bringup/config/arms-kh.json" -p mock:=false -p allow_hardware:=true ;;
 base) exec ros2 run luxi_device_adapters r680_adapter --ros-args -p mock:=false -p allow_hardware:=true -p interface:=luxi_base -p require_left_grip:=false -p shared_enable:=true ;;
 lift)
  source "$ROOT/Project/robot_bringup/config/lift.env"
  exec ros2 run luxi_device_adapters lift_adapter --ros-args -p allow_hardware:=true -p shared_enable:=true \
   -p "allow_motion:=$LIFT_ALLOW_MOTION" -p "interface:=$LIFT_INTERFACE" \
   -p "transport:=${LIFT_TRANSPORT:-can}" -p "serial_port:=${LIFT_SERIAL_PORT:-}" \
   -p "host_min_um:=${LIFT_HOST_MIN_UM:--100000}" -p "host_max_um:=${LIFT_HOST_MAX_UM:-200000}" \
   -p "node_id:=$LIFT_NODE_ID" -p "margin_um:=$LIFT_MARGIN_UM" ;;
 program)
  source "$ROOT/Project/luxi_web_ctrl/scripts/collection_env.sh"
  exec ros2 launch "$ROOT/Project/luxi_web_ctrl/scripts/program.launch.xml" \
   "directory:=$COLLECTION_DIRECTORY" "minimum_free_gib:=$COLLECTION_MIN_FREE_GIB" \
   "camera_config:=$COLLECTION_CAMERA_CONFIG" ;;
 *) exit 2;;
esac
