#!/usr/bin/env bash
# Only configure the two arm buses; leave the running base bus alone.
set -euo pipefail
if pgrep -x openarm_adapter >/dev/null; then
 echo 'Stop existing openarm_adapter before CAN setup' >&2
 exit 1
fi
for iface in can1 can2; do
 ip link show "$iface" >/dev/null
 if ip -details link show "$iface" | grep -q 'bitrate 1000000 '; then
  if ip -details link show "$iface" | grep -q 'dbitrate 5000000' && [[ $(cat "/sys/class/net/$iface/mtu") == 72 ]]; then
   ip link set "$iface" up
   continue
  fi
 fi
 ip link set "$iface" down
 ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
 ip link set "$iface" up
done
