#!/usr/bin/env bash
# Local SDK prefix only; no sudo, kernel patches, USB resets or service changes.
# Optional positional arguments provide pre-downloaded SDK and JSON archives.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORK="$ROOT/build/depth-sdk"
mkdir -p "$WORK" "$ROOT/log"
if [[ ${1:-} == --prebuilt ]]; then
 [[ $(uname -m) == aarch64 ]] || { echo 'Prebuilt packages require aarch64' >&2; exit 2; }
 RUNTIME=${2:-$WORK/librealsense2-runtime.deb}
 DEVELOPMENT=${3:-$WORK/librealsense2-dev.deb}
 BASE=https://librealsense.realsenseai.com/Debian/apt-repo/pool/jammy/main
 [[ -f "$RUNTIME" ]] || curl -fL --retry 2 -o "$RUNTIME" "$BASE/librealsense2_2.56.5-0~realsense.6682_arm64.deb"
 [[ -f "$DEVELOPMENT" ]] || curl -fL --retry 2 -o "$DEVELOPMENT" "$BASE/librealsense2-dev_2.56.5-0~realsense.6682_arm64.deb"
 printf '%s  %s\n' 5a3c1a77257bc01dc6a886a59495916de806277fdd9c065ee0ac09ad175d0bcf "$RUNTIME" \
  21a403864ed0bf0d94683b927c315309dfbfd71e4a16eefa2addaf9ba14248f8 "$DEVELOPMENT" | sha256sum -c -
 mkdir -p "$WORK/debs" "$WORK/install"
 dpkg-deb -x "$RUNTIME" "$WORK/debs"
 dpkg-deb -x "$DEVELOPMENT" "$WORK/debs"
 cp -a "$WORK/debs/usr/." "$WORK/install/"
 if ldd "$WORK/install/lib/aarch64-linux-gnu/librealsense2.so" | grep -q 'not found'; then
  echo 'SDK runtime dependencies missing; inspect ldd output or build SDK from source' >&2; exit 2
 fi
 echo "SDK extracted to $WORK/install without system package installation. Rebuild with tool/build.sh."
 exit 0
fi
SDK=${1:-$WORK/librealsense-v2.56.5.tar.gz}
JSON_ARCHIVE=${2:-$WORK/json-v3.11.3.tar.gz}
if [[ ! -f "$SDK" ]]; then
 curl -fL --retry 2 -o "$SDK" https://codeload.github.com/realsenseai/librealsense/tar.gz/refs/tags/v2.56.5
fi
if [[ ! -f "$JSON_ARCHIVE" ]]; then
 curl -fL --retry 2 -o "$JSON_ARCHIVE" https://codeload.github.com/nlohmann/json/tar.gz/refs/tags/v3.11.3
fi
printf '%s  %s\n' 58b2029eb7179afc7ea893c25be38a65b42c5b6d27330c8c611bc0e0a4ac5a85 "$SDK" \
 0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406 "$JSON_ARCHIVE" | sha256sum -c -
mkdir -p "$WORK/source" "$WORK/json"
tar -xzf "$SDK" -C "$WORK/source" --strip-components=1
tar -xzf "$JSON_ARCHIVE" -C "$WORK/json" --strip-components=1
# rsutils also references this upstream build-tree include path directly.
mkdir -p "$WORK/cmake/third-party/json"
cp -a "$WORK/json/include" "$WORK/cmake/third-party/json/"
# Upstream unconditionally clones JSON at configure time. Use the verified local
# copy instead, making offline builds reproducible. Only generated source changes.
cat > "$WORK/source/CMake/external_json.cmake" <<'EOF'
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../json" "${CMAKE_BINARY_DIR}/third-party/json/build")
EOF
cmake -S "$WORK/source" -B "$WORK/cmake" -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_INSTALL_PREFIX="$WORK/install" -DBUILD_SHARED_LIBS=ON \
 -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF \
 -DBUILD_TOOLS=OFF -DBUILD_WITH_DDS=OFF -DBUILD_GLSL_EXTENSIONS=OFF -DHWM_OVER_XU=ON
cmake --build "$WORK/cmake" -j2
cmake --install "$WORK/cmake"
echo "SDK installed under $WORK/install. Rebuild with tool/build.sh."
