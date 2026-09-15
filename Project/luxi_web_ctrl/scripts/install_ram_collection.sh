#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ $EUID -ne 0 ]] || { echo 'Run as the ordinary robot user'; exit 1; }
[[ $(wc -l < /proc/swaps) -eq 1 ]] || { echo 'RAM-only mode requires swap disabled'; exit 1; }
mkdir -p "$ROOT/build/web-systemd"
cat > "$ROOT/build/web-systemd/run-luxi_dataset.mount" <<UNIT
[Unit]
Description=LUXI bounded dataset RAM staging
[Mount]
What=tmpfs
Where=/run/luxi_dataset
Type=tmpfs
Options=rw,nosuid,nodev,noexec,size=2G,mode=0750,uid=$(id -u),gid=$(id -g)
[Install]
WantedBy=multi-user.target
UNIT
sudo install -m 644 "$ROOT/build/web-systemd/run-luxi_dataset.mount" /etc/systemd/system/
for unit in luxi-web.service luxi-program.service; do
 sudo mkdir -p "/etc/systemd/system/$unit.d"
 printf '[Unit]\nRequires=run-luxi_dataset.mount\nAfter=run-luxi_dataset.mount\n' | sudo tee "/etc/systemd/system/$unit.d/collection-ram.conf" >/dev/null
done
sudo systemctl daemon-reload
sudo systemctl enable --now run-luxi_dataset.mount
mkdir -p /run/luxi_dataset/episodes
cat > "$ROOT/Project/robot_bringup/config/collection.env" <<'CONFIG'
COLLECTION_DIRECTORY=/run/luxi_dataset/episodes
COLLECTION_MIN_FREE_GIB=0.25
COLLECTION_RAM=true
CONFIG
echo 'RAM staging installed (2 GiB). Restart web/program while motors are disabled to load it.'
