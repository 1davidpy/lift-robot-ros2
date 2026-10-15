#!/usr/bin/env bash
# Install an independent HTTP gateway. Does not start camera or motion nodes.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
PORT=${1:-18030}
[[ $PORT =~ ^[0-9]+$ && ${#PORT} -le 5 ]] || exit 2
(( 10#$PORT >= 1024 && 10#$PORT <= 65535 )) || exit 2
SERVICE_USER=$(id -un)
UNIT=$(mktemp)
trap 'rm -f "$UNIT"' EXIT
cat > "$UNIT" <<EOF
[Unit]
Description=LUXI aligned RGBD HTTP gateway
After=network-online.target
[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=ROS_LOG_DIR=$ROOT/log/ros
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash; source "$ROOT/build/install/setup.bash"; exec "$ROOT/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl" --ros-args -r __node:=rgbd_gateway -p host:=0.0.0.0 -p port:=$PORT -p mock:=false -p base_enabled:=true -p directory:="$ROOT/data/interface_gateway"'
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
EOF
sudo install -m 0644 "$UNIT" /etc/systemd/system/luxi-rgbd-gateway.service
sudo systemctl daemon-reload
echo "Installed luxi-rgbd-gateway.service on port $PORT (not started or enabled)"
