#!/usr/bin/env bash
# Installs a persistent dashboard; optional robot unit starts only on next boot.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
MODE=${1:---web-only}
[[ $# -le 1 && ( "$MODE" == --web-only || "$MODE" == --hardware ) ]] || { echo 'Usage: install_service.sh [--web-only|--hardware]'; exit 2; }
[[ $EUID -ne 0 ]] || { echo 'Run as the ordinary robot user, not sudo'; exit 1; }
[[ "$ROOT" != *' '* ]] || { echo 'Service installation requires a path without spaces'; exit 1; }
[[ -x "$ROOT/build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl" ]] || { echo 'Build first'; exit 1; }
mkdir -p "$ROOT/log" "$ROOT/data/real" "$ROOT/build/web-systemd"
if [[ ! -f "$ROOT/log/web-control.token" ]]; then
 (umask 077; openssl rand -hex 24 > "$ROOT/log/web-control.token")
fi
chmod 600 "$ROOT/log/web-control.token"
USER_NAME=$(id -un)
cat > "$ROOT/build/web-systemd/luxi-web.service" <<UNIT
[Unit]
Description=LUXI LAN control dashboard
Wants=network-online.target
After=network-online.target
[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStart=$ROOT/Project/luxi_web_ctrl/scripts/run_web.sh
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=25
[Install]
WantedBy=multi-user.target
UNIT
cat > "$ROOT/build/web-systemd/luxi-robot.service" <<UNIT
[Unit]
Description=LUXI real OpenArm and R680 acquisition stack
Wants=luxi-web.service
After=luxi-web.service
[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStartPre=+$ROOT/Project/luxi_web_ctrl/scripts/prepare_can.sh
ExecStart=$ROOT/tool/run.sh mock:=false allow_hardware:=true base_enabled:=true web_enabled:=false config:=$ROOT/Project/robot_bringup/config/arms-kh.json no_cameras:=true require_cameras:=false directory:=$ROOT/data/real
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=30
[Install]
WantedBy=multi-user.target
UNIT
sudo install -m 644 "$ROOT/build/web-systemd/luxi-web.service" /etc/systemd/system/luxi-web.service
sudo install -m 644 "$ROOT/build/web-systemd/luxi-robot.service" /etc/systemd/system/luxi-robot.service
sudo systemctl daemon-reload
sudo systemctl enable luxi-web.service
if [[ "$MODE" == --hardware ]]; then sudo systemctl enable luxi-robot.service; fi
sudo systemctl start luxi-web.service
printf '%s\n' 'Web dashboard enabled on boot: http://BOARD_LAN_IP:8000' \
 "Read control token locally: cat $ROOT/log/web-control.token"
if [[ "$MODE" == --hardware ]]; then
 echo 'Hardware startup enabled for NEXT BOOT; no motors enabled now. Cameras remain off.'
else
 echo 'Only web service installed/enabled; existing hardware enable setting was not changed.'
fi
