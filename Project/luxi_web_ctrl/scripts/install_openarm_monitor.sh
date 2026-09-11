#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ "$ROOT" != *' '* ]]
mkdir -p "$ROOT/build/web-systemd"
cat > "$ROOT/build/web-systemd/luxi-openarm-monitor.service" <<UNIT
[Unit]
Description=LUXI OpenArm feedback monitor (no motor enable)
After=network-online.target luxi-web.service
Conflicts=luxi-robot.service
[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStartPre=+$ROOT/Project/luxi_web_ctrl/scripts/prepare_openarm_can.sh
ExecStart=$ROOT/Project/luxi_web_ctrl/scripts/run_openarm_monitor.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=10
[Install]
WantedBy=multi-user.target
UNIT
sudo install -m 644 "$ROOT/build/web-systemd/luxi-openarm-monitor.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now luxi-openarm-monitor.service
