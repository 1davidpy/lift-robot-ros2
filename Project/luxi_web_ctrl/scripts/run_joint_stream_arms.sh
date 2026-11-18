#!/usr/bin/env bash
set -eo pipefail
SDK_ROOT=/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
source /opt/ros/humble/setup.bash
source "$SDK_ROOT/build/install/setup.bash"
export ROS_LOG_DIR="$SDK_ROOT/log/ros"
exec "$SDK_ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter" --ros-args \
  -p "config:=$SDK_ROOT/Project/robot_bringup/config/arms-kh.json" \
  -p mock:=false -p allow_hardware:=true -p "monitor_only:=${LUXI_ACQUISITION_ONLY:-true}" \
  -p "joint_stream_bind:=0.0.0.0" -p joint_stream_port:=18034
