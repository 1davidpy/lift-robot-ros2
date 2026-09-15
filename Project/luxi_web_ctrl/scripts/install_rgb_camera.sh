#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ $EUID -ne 0 ]] || { echo "Run as the ordinary robot user"; exit 1; }
[[ $# == 2 || $# == 3 ]] || { echo 'Usage: install_rgb_camera.sh head_rgb|left_wrist_rgb|right_wrist_rgb SERIAL [hikrobot|realsense]'; exit 2; }
CAMERA_ID=$1
case "$CAMERA_ID" in head_rgb|left_wrist_rgb|right_wrist_rgb) ;; *) exit 2;; esac
[[ "$2" =~ ^[a-zA-Z0-9]+$ && "$ROOT" != *' '* ]]
CAMERA_BACKEND=${3:-hikrobot}
[[ "$CAMERA_BACKEND" == hikrobot || "$CAMERA_BACKEND" == realsense ]] || exit 2
mkdir -p "$ROOT/build/web-systemd" "$ROOT/log/ros"
cat > "$ROOT/build/web-systemd/luxi-rgb-$CAMERA_ID.service" <<UNIT
[Unit]
Description=LUXI RGB camera $CAMERA_ID
After=network-online.target luxi-web.service
Conflicts=luxi-robot.service
[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=CAMERA_SERIAL=$2
Environment=CAMERA_BACKEND=$CAMERA_BACKEND
ExecStart=$ROOT/Project/luxi_web_ctrl/scripts/run_rgb_camera.sh $CAMERA_ID
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
UNIT
sudo install -m 644 "$ROOT/build/web-systemd/luxi-rgb-$CAMERA_ID.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now "luxi-rgb-$CAMERA_ID.service"
