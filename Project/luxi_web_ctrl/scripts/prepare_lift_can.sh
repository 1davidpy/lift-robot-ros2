#!/usr/bin/env bash
# Only configure the already identified fourth adapter port; never reset CH1-3.
set -euo pipefail
exec 9>/run/lock/luxi-can-prepare.lock
flock -w 30 9
IFACE=luxi_lift
[[ -d /sys/class/net/$IFACE ]] || { echo 'CH4/luxi_lift absent; identify adapter before starting lift' >&2; exit 1; }
# Existing managed units may own other ports. Do not run kh_can_setup --apply.
ip link set dev "$IFACE" down
ip link set dev "$IFACE" type can bitrate 500000 sample-point 0.778 fd off \
  listen-only off berr-reporting on restart-ms 0
ip link set dev "$IFACE" txqueuelen 16
ip link set dev "$IFACE" up
