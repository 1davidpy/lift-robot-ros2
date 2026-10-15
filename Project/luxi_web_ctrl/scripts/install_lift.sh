#!/usr/bin/env bash
# Install only the new unit and expanded management allowlist; do not start it.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ $EUID -ne 0 && "$ROOT" != *' '* ]]
USER_NAME=$(id -un)
mkdir -p "$ROOT/build/web-systemd"
cat > "$ROOT/build/web-systemd/luxi-lift.service" <<EOF
[Unit]
Description=LUXI STM32 lift and Quest grips (configured UART/CAN transport)
After=network-online.target luxi-openarm-monitor.service luxi-quest.service
Conflicts=luxi-robot.service
StartLimitIntervalSec=0
[Service]
User=$USER_NAME
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStartPre=+/bin/bash $ROOT/Project/luxi_web_ctrl/scripts/prepare_lift.sh
ExecStart=/bin/bash $ROOT/Project/luxi_web_ctrl/scripts/run_managed.sh lift
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStartSec=40
TimeoutStopSec=5
[Install]
WantedBy=multi-user.target
EOF
sudo install -d -m 755 /usr/local/libexec
sudo install -o root -g root -m 755 "$ROOT/Project/luxi_web_ctrl/scripts/service_control.py" /usr/local/libexec/luxi-service-control
sudo install -m 644 "$ROOT/build/web-systemd/luxi-lift.service" /etc/systemd/system/
sudo systemctl daemon-reload
printf '%s\n' 'Lift service installed but not started or enabled at boot. Start it explicitly after hardware commissioning.'
