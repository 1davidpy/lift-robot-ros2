#!/usr/bin/env bash
# Install only. Starting capture is a separate, explicit operation.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
CAMERA_ID=${1:?Usage: install_depth_camera.sh CAMERA_ID SERIAL}
SERIAL=${2:?Usage: install_depth_camera.sh CAMERA_ID SERIAL}
case "$CAMERA_ID" in head_rgb|left_wrist_rgb|right_wrist_rgb) ;; *) exit 2;; esac
[[ $SERIAL =~ ^[0-9]+$ ]] || exit 2
SERVICE_USER=$(id -un)
ALIGN_TO_COLOR=${ALIGN_TO_COLOR:-true}
case "$ALIGN_TO_COLOR" in true|false) ;; *) exit 2;; esac
CONFLICTS=
if [[ $ALIGN_TO_COLOR == true ]]; then
  CONFLICTS="Conflicts=luxi-rgb-$CAMERA_ID.service luxi-robot.service
After=luxi-rgb-$CAMERA_ID.service"
fi
UNIT=$(mktemp)
trap 'rm -f "$UNIT"' EXIT
cat > "$UNIT" <<EOF
[Unit]
Description=LUXI RealSense RGBD depth $CAMERA_ID
After=network-online.target luxi-web.service
$CONFLICTS
[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=CAMERA_SERIAL=$SERIAL
Environment=ALIGN_TO_COLOR=$ALIGN_TO_COLOR
ExecStart=/bin/bash $ROOT/Project/luxi_web_ctrl/scripts/run_depth_camera.sh $CAMERA_ID
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
EOF
sudo install -m 0644 "$UNIT" "/etc/systemd/system/luxi-depth-$CAMERA_ID.service"
sudo systemctl daemon-reload
sudo install -d -m 0755 /usr/local/libexec
sudo install -m 0755 "$ROOT/Project/luxi_web_ctrl/scripts/service_control.py" /usr/local/libexec/luxi-service-control
echo "Installed luxi-depth-$CAMERA_ID.service (not started or enabled)"
