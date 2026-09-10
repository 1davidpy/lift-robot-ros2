#!/usr/bin/env bash
# Startup orchestration only; ROS/CAN business logic remains in C++.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:---mock}
CHECK=${2:-}
if [[ "$MODE" == --help ]]; then
  echo 'Usage: start_robot.sh --mock|--hardware [--check]'
  echo '--hardware: KH CH1/2 OpenArm + CH3 R680, cameras disabled, no automatic motion enable.'
  exit 0
fi
[[ $# -le 2 && ( "$MODE" == --mock || "$MODE" == --hardware ) && ( -z "$CHECK" || "$CHECK" == --check ) ]] || { echo 'Use --help' >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash && -f "$ROOT/build/install/setup.bash" ]] || { echo 'Run tool/build.sh first' >&2; exit 1; }
for entry in luxi_device_adapters/r680_adapter luxi_device_adapters/openarm_adapter luxi_quest_receiver/quest_receiver luxi_web_ctrl/web_ctrl; do
  package=${entry%%/*}
  [[ -x "$ROOT/build/install/$package/lib/$entry" ]] || { echo "Missing executable: $entry" >&2; exit 1; }
done
WEB_ENABLED=true
WEB_SERVICE=false
HTTP_PORT=8000
QUEST_PORT=5006
if systemctl is-active --quiet luxi-web.service; then
  WEB_SERVICE=true
  if [[ "$MODE" == --hardware ]]; then
    WEB_ENABLED=false
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
  else
    export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1
    HTTP_PORT=18000
    QUEST_PORT=15006
  fi
fi
# Do not reconfigure buses while another controller is running.
for name in openarm_adapter r680_adapter quest_receiver data_server web_ctrl; do
  if [[ "$name" == web_ctrl && "$WEB_SERVICE" == true ]]; then continue; fi
  if pgrep -x "$name" >/dev/null; then echo "Stop existing $name before starting another stack" >&2; exit 1; fi
done
for spec in "udp $QUEST_PORT" "tcp $HTTP_PORT"; do
  read -r protocol port <<< "$spec"
  if [[ "$protocol" == tcp && "$WEB_ENABLED" == false ]]; then continue; fi
  if [[ "$protocol" == udp ]]; then output=$(ss -H -lun "sport = :$port"); else output=$(ss -H -ltn "sport = :$port"); fi
  [[ -z "$output" ]] || { echo "$protocol port $port is busy" >&2; exit 1; }
done
MODULE="$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/kcan.ko"
SETUP="$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/kh_can_setup"
if [[ "$MODE" == --hardware ]]; then
  [[ -f "$MODULE" ]] || { echo 'Run tool/build_kh_driver.sh first' >&2; exit 1; }
  [[ "$(modinfo -F vermagic "$MODULE")" == "$(uname -r) "* ]] || { echo 'Rebuild kcan for this kernel' >&2; exit 1; }
  [[ -n "$(lsusb -d 395e:0020)" ]] || { echo 'KH X4 USB device not found' >&2; exit 1; }
  [[ -f "$ROOT/Project/robot_bringup/config/arms-kh.json" && -f "$ROOT/device/R680/channels-r680.json" ]] || exit 1
  if [[ -d /sys/module/kcan ]]; then
    [[ "$(cat /sys/module/kcan/version)" == "$(modinfo -F version "$MODULE")" ]] || { echo 'Loaded kcan version differs; inspect before restarting' >&2; exit 1; }
    "$SETUP" "$ROOT/device/R680/channels-r680.json"
  fi
fi
if [[ "$CHECK" == --check ]]; then
  echo "Preflight OK ($MODE). No module loaded, no CAN changes, no nodes started."
  echo 'This checks local prerequisites only, not motor wiring, camera availability or motion.'
  exit 0
fi
mkdir -p "$ROOT/log"
exec 9>"$ROOT/log/start_robot.lock"
flock -n 9 || { echo 'Another start_robot.sh instance is running' >&2; exit 1; }
if [[ "$MODE" == --mock ]]; then
  echo "Starting MOCK robot + three mock cameras: http://127.0.0.1:$HTTP_PORT; Quest UDP $QUEST_PORT"
  exec "$ROOT/tool/run.sh" mock:=true base_enabled:=true minimum_free_gib:=0.2 "directory:=$ROOT/data/mock" "http_port:=$HTTP_PORT" "quest_port:=$QUEST_PORT"
fi
sudo -v
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
if [[ ! -d /sys/module/kcan ]]; then sudo insmod "$MODULE"; fi
"$SETUP" "$ROOT/device/R680/channels-r680.json"
sudo "$SETUP" "$ROOT/device/R680/channels-r680.json" --apply
printf '%s\n' 'Starting REAL arms and R680; cameras OFF; motion permission remains disabled.' 'Local control: http://127.0.0.1:8000 ; Quest destination: board WLAN IPv4, UDP 5006'
exec "$ROOT/tool/run.sh" mock:=false allow_hardware:=true base_enabled:=true \
  "config:=$ROOT/Project/robot_bringup/config/arms-kh.json" \
  no_cameras:=true require_cameras:=false "directory:=$ROOT/data/real" "web_enabled:=$WEB_ENABLED"
