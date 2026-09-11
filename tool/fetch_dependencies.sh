#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$ROOT/build/downloads"
while IFS=$'	' read -r destination url digest; do
    target="$ROOT/$destination"
    if [[ -d "$target" ]]; then
        [[ -f "$target/.luxi-source" && $(cat "$target/.luxi-source") == "$digest" ]] || { echo "Unrecognized dependency tree: $target" >&2; exit 1; }
        continue
    fi
    archive="$ROOT/build/downloads/$digest.tar.gz"
    curl --fail --location --retry 3 "$url" -o "$archive"
    printf '%s  %s\n' "$digest" "$archive" | sha256sum --check -
    mkdir -p "$target"
    tar -xzf "$archive" --strip-components=1 -C "$target"
    if [[ $destination == device/openarm/openarm_can ]]; then
        patch --directory="$target" -p1 < "$ROOT/device/patches/openarm_can-transmit-errors.patch"
    fi
    printf '%s\n' "$digest" > "$target/.luxi-source"
done < "$ROOT/3part/artifacts.lock"
