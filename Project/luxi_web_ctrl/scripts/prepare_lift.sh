#!/usr/bin/env bash
# Select only the lift transport. UART must never run CAN interface setup.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
source "$ROOT/Project/robot_bringup/config/lift.env"
case "${LIFT_TRANSPORT:-can}" in
  can) exec /bin/bash "$ROOT/Project/luxi_web_ctrl/scripts/prepare_lift_can.sh" ;;
  uart)
    [[ -n "${LIFT_SERIAL_PORT:-}" && -c "$LIFT_SERIAL_PORT" ]] || {
      echo 'Configured lift USB serial device absent; reconnect its identified USB port.' >&2
      exit 1
    }
    ;;
  *) echo 'Unknown lift transport' >&2; exit 1 ;;
esac
