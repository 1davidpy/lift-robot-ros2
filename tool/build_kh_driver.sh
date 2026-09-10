#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ARCHIVE="$ROOT/device/KH-UCANFD-B2B-X4/SDK/KH-UCANFD_Linux_SDK.zip"
EXPECTED=0859afc67dc63f232d64a8815a4c2a7bcf7129298c48748a70174647a4a6508c
[[ $(sha256sum "$ARCHIVE" | cut -d ' ' -f1) == "$EXPECTED" ]] || { echo 'SDK SHA256 mismatch' >&2; exit 1; }
KERNEL_HEADERS=${1:-/usr/src/linux-headers-6}
RELEASE=$(cat "$KERNEL_HEADERS/include/config/kernel.release")
[[ "$RELEASE" == "$(uname -r)" ]] || { echo 'Kernel headers do not match running kernel' >&2; exit 1; }
mkdir -p "$ROOT/build/kh-sdk" "$ROOT/log"
if [[ ! -d "$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2" ]]; then
  unzip -q "$ARCHIVE" -d "$ROOT/build/kh-sdk"
fi
KERNEL_COPY="$ROOT/build/kernel-$RELEASE"
[[ -d "$KERNEL_COPY" ]] || cp -a "$KERNEL_HEADERS" "$KERNEL_COPY"
# Build only the missing host tools. The board header package omits the
# source Kbuild needed by full modules_prepare; its generated headers exist.
cat > "$KERNEL_COPY/luxi-tools.mk" <<'MAKE'
.PHONY: luxi_modpost
luxi_modpost: archprepare
	$(Q)$(MAKE) $(build)=scripts/mod
MAKE
make -C "$KERNEL_COPY" -f Makefile -f luxi-tools.mk luxi_modpost -j2 > "$ROOT/log/kh-kernel-prepare.log" 2>&1
make -C "$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2/driver" netdev KERNEL_LOCATION="$KERNEL_COPY" -j2 > "$ROOT/log/kh-driver-build.log" 2>&1
modinfo "$ROOT/build/kh-sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/kcan.ko"
echo 'Built only. Loading/installing is a separate deployment step; see device/KH-UCANFD-B2B-X4/README.md.'
