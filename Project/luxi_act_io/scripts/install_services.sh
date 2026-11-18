#!/usr/bin/env bash
# Install/start only the two new ingress services. No camera or motor lifecycle calls.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
SERVICE_USER=$(id -un)
for binary in act_rgb_http act_action_udp; do
  test -x "$ROOT/build/install/luxi_act_io/lib/luxi_act_io/$binary"
done
mkdir -p "$ROOT/log/ros"
UNIT_DIR=$(mktemp -d)
trap 'rm -rf "$UNIT_DIR"' EXIT
for spec in 'luxi-act-rgb-http act_rgb_http 18040' 'luxi-act-action-udp act_action_udp 18041'; do
  read -r service binary port <<< "$spec"
  # Never overwrite a pre-existing unrelated unit with a matching name.
  if [[ -f /etc/systemd/system/$service.service ]] && ! grep -q 'Managed by luxi_act_io' "/etc/systemd/system/$service.service"; then
    echo "Refusing to overwrite existing unit: $service" >&2; exit 1
  fi
  cat > "$UNIT_DIR/$service.service" <<EOF
# Managed by luxi_act_io
[Unit]
Description=LUXI ACT $binary (no motor execution)
After=network-online.target
Wants=network-online.target
[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$ROOT
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=ROS_LOG_DIR=$ROOT/log/ros
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash; source "$ROOT/build/install/setup.bash"; exec "$ROOT/build/install/luxi_act_io/lib/luxi_act_io/$binary" --ros-args -p host:=0.0.0.0 -p port:=$port'
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=10
NoNewPrivileges=true
[Install]
WantedBy=multi-user.target
EOF
done
sudo -n install -m 0644 "$UNIT_DIR/"*.service /etc/systemd/system/
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now luxi-act-rgb-http.service luxi-act-action-udp.service
sudo -n systemctl --no-pager --full status luxi-act-rgb-http.service luxi-act-action-udp.service
