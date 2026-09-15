#!/usr/bin/env bash
# Sourced after ROOT is set. A missing RAM mount must never fall back to the SD card.
COLLECTION_DIRECTORY="$ROOT/data/real"
COLLECTION_MIN_FREE_GIB=20.0
COLLECTION_RAM=false
COLLECTION_CAMERA_CONFIG="$ROOT/Project/robot_bringup/config/cameras-realsense.json"
if [[ -f "$ROOT/Project/robot_bringup/config/collection.env" ]]; then
 source "$ROOT/Project/robot_bringup/config/collection.env"
fi
if [[ $COLLECTION_RAM == true ]]; then
 [[ $(findmnt -n -o FSTYPE -T "$COLLECTION_DIRECTORY" 2>/dev/null || true) == tmpfs ]] || {
  echo "Dataset RAM mount unavailable: $COLLECTION_DIRECTORY" >&2; exit 1;
 }
 [[ $(wc -l < /proc/swaps) -eq 1 ]] || { echo 'Disable swap before RAM-only recording' >&2; exit 1; }
fi
mkdir -p "$COLLECTION_DIRECTORY"
