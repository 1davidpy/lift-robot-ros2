#!/usr/bin/env bash
# Fetch the publicly shared EB435i SDK bundle linked by the supplier's documentation.
set -euo pipefail
DEVICE_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ARCHIVE="$DEVICE_DIR/sdk-package-20250418.zip"
SDK_ZIP="$DEVICE_DIR/SDK_Mv3dRgbd_Linux_V1.4.0.3_250213.zip"
if [[ ! -f "$ARCHIVE" ]]; then
 curl --fail --location --retry 3 --connect-timeout 20 \
  'https://drive.usercontent.google.com/download?id=1CULy0L3frJedtBTYsNHOD8DzctW-zUKp&export=download&confirm=t' \
  -o "$ARCHIVE.part"
 mv "$ARCHIVE.part" "$ARCHIVE"
fi
printf '%s  %s\n' '51473133b1f28e6507f87c56a017d5e64c3d3fa6273756b719002535780edff2' "$ARCHIVE" | sha256sum --check -
python3 - "$DEVICE_DIR" <<'PY'
import hashlib, pathlib, sys, zipfile
root=pathlib.Path(sys.argv[1])
with zipfile.ZipFile(root/'sdk-package-20250418.zip') as archive:
 name=next(n for n in archive.namelist() if n.endswith('/SDK_Mv3dRgbd_Linux_V1.4.0.3_250213.zip'))
 data=archive.read(name)
expected='e9f641c31d90224350222dd3a2cf6b5a8b0d53d292334ce696b6bd5caded0bff'
assert hashlib.sha256(data).hexdigest()==expected,'SDK checksum mismatch'
sdk_zip=root/'SDK_Mv3dRgbd_Linux_V1.4.0.3_250213.zip'
sdk_zip.write_bytes(data)
target=root/'sdk-1.4'
marker=target/'.luxi-source'
if target.exists():
 assert marker.is_file() and marker.read_text().strip()==expected,'Unrecognized SDK directory; preserve and inspect it first'
else:
 with zipfile.ZipFile(sdk_zip) as archive:
  assert all(not pathlib.PurePosixPath(n).is_absolute() and '..' not in pathlib.PurePosixPath(n).parts for n in archive.namelist())
  archive.extractall(target)
 marker.write_text(expected+'\n')
libs=target/'MVCC/Libs/aarch64'
for name in ['libMvCameraControl','libMVGigEVisionSDK','libMvUsb3vTL','libMvCameraControlWrapper']:
 link=libs/(name+'.so')
 if not link.exists():
  versions=list(libs.glob(name+'.so.*'))
  assert len(versions)==1, 'Ambiguous dependency version'
  link.symlink_to(versions[0].name)
PY
readelf -h "$DEVICE_DIR/sdk-1.4/Libs/aarch64/libMv3dRgbd.so" | grep -q AArch64
printf '%s\n' "SDK ready: $DEVICE_DIR/sdk-1.4" 'Source sdk-env.sh to load its project-local runtime.'
