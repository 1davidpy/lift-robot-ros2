#!/usr/bin/env bash
# Source this file for the project-local ARM64 MV3D RGB-D 1.4 runtime.
HIK_SDK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/sdk-1.4" && pwd)
export MVCAM_COMMON_RUNENV="$HIK_SDK_ROOT/MVCC/Libs"
export MV3D_RGBD_COMMON_RUNENV="$HIK_SDK_ROOT/Libs"
export MV3D_RGBD_SDK_PATH="$HIK_SDK_ROOT"
export LD_LIBRARY_PATH="$HIK_SDK_ROOT/Libs/aarch64:$HIK_SDK_ROOT/MVCC/Libs/aarch64:${LD_LIBRARY_PATH:-}"
export SDK_LIBRARY="${SDK_LIBRARY:-$HIK_SDK_ROOT/Libs/aarch64/libMv3dRgbd.so}"
