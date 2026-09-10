#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if ip link show luxi_r680_test >/dev/null 2>&1; then
  echo 'Dedicated test interface already exists; refusing to modify it' >&2
  exit 1
fi
sudo modprobe vcan
sudo ip link add luxi_r680_test type vcan
trap 'sudo ip link delete luxi_r680_test' EXIT
sudo ip link set luxi_r680_test mtu 16 up
"$ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_socket_probe"
