#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
CAMERA_ID=${1:-head_rgb}
case "$CAMERA_ID" in head_rgb|left_wrist_rgb|right_wrist_rgb) ;; *) exit 2;; esac
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
source "$ROOT/device/HIK-EB435I/sdk-env.sh"
mkdir -p "$ROOT/log/cameras/$CAMERA_ID"
cd "$ROOT/log/cameras/$CAMERA_ID"
exec "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/hikrobot_camera" --ros-args \
 -r "__node:=camera_$CAMERA_ID" -p mock:=false -p require_usb:=true -p "camera_id:=$CAMERA_ID" \
 -p "serial:=${CAMERA_SERIAL:?Set CAMERA_SERIAL}" -p "sdk_library:=$SDK_LIBRARY"
