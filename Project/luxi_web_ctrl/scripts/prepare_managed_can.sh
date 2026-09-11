#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
exec 9>/run/lock/luxi-can-prepare.lock
flock -w 30 9
MODULE="$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/kcan.ko"
modprobe can_raw
modprobe can_dev
if [[ ! -d /sys/module/kcan ]]; then insmod "$MODULE"; fi
for attempt in {1..10}; do
 if [[ -d /sys/class/net/luxi_left && -d /sys/class/net/luxi_right && -d /sys/class/net/luxi_base ]]; then
  for iface in luxi_left luxi_right luxi_base; do ip link set "$iface" up; done
  exit 0
 fi
 if "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/kh_can_setup" "$ROOT/device/R680/channels-r680.json" >/dev/null 2>&1; then
  "$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/kh_can_setup" "$ROOT/device/R680/channels-r680.json" --apply
  for iface in luxi_left luxi_right luxi_base; do
   ip link set "$iface" down
   ip link set "$iface" type can restart-ms 100
   ip link set "$iface" up
  done
  exit 0
 fi
 sleep 1
done
echo 'USB CAN absent; acquisition will retry independently of web' >&2
exit 1
