#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $# == 1 ]] || { echo "Usage: bash tool/install_hikrobot_sdk.sh /extracted/sdk/library-directory" >&2; exit 2; }
source_dir=$(realpath "$1")
lib="$source_dir/libMv3dRgbd.so"
[[ -f "$lib" ]] || { echo "MV3D RGB-D library missing (generic MVS is not sufficient)" >&2; exit 1; }
readelf -h "$lib" | grep -q AArch64 || { echo 'SDK must be AArch64 ELF' >&2; exit 1; }
[[ ! -e "$ROOT/device/sdk/hikrobot" ]] || { echo 'Destination already exists; preserve it and select a separate version.' >&2; exit 1; }
mkdir -p "$ROOT/device/sdk"
cp -a "$source_dir" "$ROOT/device/sdk/hikrobot"
mkdir -p "$ROOT/log/sdk"
sha256sum "$ROOT/device/sdk/hikrobot/libMv3dRgbd.so" > "$ROOT/log/sdk/hikrobot.sha256"
echo "Installed SDK libraries. Set sdk_library:=$ROOT/device/sdk/hikrobot/libMv3dRgbd.so and LD_LIBRARY_PATH to its dependency directory. No camera was opened."
