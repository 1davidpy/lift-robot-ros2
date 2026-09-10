#!/usr/bin/env bash
# Run only by the optional hardware systemd unit, as root.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
for process in openarm_adapter r680_adapter quest_receiver; do
  if pgrep -x "$process" >/dev/null; then echo "Stop existing $process before CAN setup" >&2; exit 1; fi
done
MODULE="$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/kcan.ko"
[[ "$(modinfo -F vermagic "$MODULE")" == "$(uname -r) "* ]]
for attempt in {1..15}; do
 if [[ -n "$(lsusb -d 395e:0020)" ]]; then break; fi
 sleep 2
done
[[ -n "$(lsusb -d 395e:0020)" ]] || { echo 'KH USB CAN absent; web remains available' >&2; exit 1; }
modprobe can
modprobe can_raw
modprobe can_dev
if [[ ! -d /sys/module/kcan ]]; then insmod "$MODULE"; fi
[[ "$(cat /sys/module/kcan/version)" == "$(modinfo -F version "$MODULE")" ]]
exec "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/kh_can_setup" "$ROOT/device/R680/channels-r680.json" --apply
