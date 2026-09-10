#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PROBE="$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/socketcan_probe"
# Dedicated virtual interface only; do not borrow or modify an existing one.
if ip link show luxi_test_can >/dev/null 2>&1; then
  echo 'luxi_test_can already exists; refusing to modify it' >&2
  exit 1
fi
sudo modprobe vcan
sudo ip link add luxi_test_can type vcan
trap 'sudo ip link delete luxi_test_can' EXIT
sudo ip link set luxi_test_can mtu 72 up
"$PROBE" --virtual-loopback luxi_test_can
