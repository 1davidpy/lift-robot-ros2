#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
CAMERA_ID=${1:-head_rgb}
case "$CAMERA_ID" in head_rgb|left_wrist_rgb|right_wrist_rgb) ;; *) exit 2;; esac
ALIGN_TO_COLOR=${ALIGN_TO_COLOR:-true}
case "$ALIGN_TO_COLOR" in true|false) ;; *) echo 'ALIGN_TO_COLOR must be true or false' >&2; exit 2;; esac
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
export LD_LIBRARY_PATH="$ROOT/build/depth-sdk/install/lib:$ROOT/build/depth-sdk/install/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
exec "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/realsense_depth" --ros-args \
 -r "__node:=depth_$CAMERA_ID" -p "camera_id:=$CAMERA_ID" \
 -p "serial:='${CAMERA_SERIAL:?Set CAMERA_SERIAL to the intended RealSense serial}'" \
 -p "width:=${DEPTH_WIDTH:-640}" -p "height:=${DEPTH_HEIGHT:-480}" -p "fps:=${DEPTH_FPS:-15}" \
 -p "align_to_color:=$ALIGN_TO_COLOR" -p "jpeg_quality:=${JPEG_QUALITY:-90}"
