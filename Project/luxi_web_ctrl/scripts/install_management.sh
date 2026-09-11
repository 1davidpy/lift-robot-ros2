#!/usr/bin/env bash
# Install durable device units and the narrowly allowlisted management helper.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ $EUID -ne 0 && "$ROOT" != *' '* ]]
export LUXI_INSTALL_ROOT="$ROOT" LUXI_INSTALL_USER="$(id -un)"
mkdir -p "$ROOT/build/web-systemd"
python3 - <<'PY'
from pathlib import Path
import os
root=os.environ['LUXI_INSTALL_ROOT']; user=os.environ['LUXI_INSTALL_USER']
out=Path(root)/'build/web-systemd'
units={'luxi-openarm-monitor':('arms',''), 'luxi-base':('base',''), 'luxi-program':('program','')}
for id,serial in [('head_rgb','00DA8573794'),('left_wrist_rgb','00DB1265540'),('right_wrist_rgb','00DA8573792')]:
 units['luxi-rgb-'+id]=(id,serial)
for unit,(id,serial) in units.items():
 pre=f'ExecStartPre=+{root}/Project/luxi_web_ctrl/scripts/prepare_managed_can.sh\n' if id in ('arms','base') else ''
 command=f'{root}/Project/luxi_web_ctrl/scripts/run_rgb_camera.sh {id}' if serial else f'{root}/Project/luxi_web_ctrl/scripts/run_managed.sh {id}'
 (out/(unit+'.service')).write_text(f'''[Unit]
Description=LUXI managed {id}
After=network-online.target luxi-web.service
Conflicts=luxi-robot.service
StartLimitIntervalSec=0
[Service]
User={user}
WorkingDirectory={root}
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=CAMERA_SERIAL={serial}
{pre}ExecStart={command}
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
KillMode=mixed
TimeoutStartSec=40
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
''')
(out/'luxi-management.sudoers').write_text(f'{user} ALL=(root) NOPASSWD: /usr/local/libexec/luxi-service-control *\n')
PY
sudo install -d -m 755 /usr/local/libexec
sudo install -o root -g root -m 755 "$ROOT/Project/luxi_web_ctrl/scripts/service_control.py" /usr/local/libexec/luxi-service-control
sudo visudo -cf "$ROOT/build/web-systemd/luxi-management.sudoers"
sudo install -o root -g root -m 440 "$ROOT/build/web-systemd/luxi-management.sudoers" /etc/sudoers.d/luxi-management
for unit in luxi-openarm-monitor luxi-base luxi-program luxi-rgb-head_rgb luxi-rgb-left_wrist_rgb luxi-rgb-right_wrist_rgb; do
 sudo install -m 644 "$ROOT/build/web-systemd/$unit.service" /etc/systemd/system/
done
sudo systemctl daemon-reload
sudo systemctl enable luxi-web.service
printf '%s\n' 'Device management installed. Start acquisition from the authenticated dashboard; motors remain disabled until explicitly enabled.'
