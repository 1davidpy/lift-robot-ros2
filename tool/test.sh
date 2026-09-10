#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/build/install/setup.bash"
set -u
export ROS_LOG_DIR="$ROOT/log/ros"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-73}
export ROS_LOCALHOST_ONLY=1
cd "$ROOT"
colcon --log-base "$ROOT/log/test" test --base-paths "$ROOT/Project" --build-base "$ROOT/build/packages" --install-base "$ROOT/build/install" --executor sequential --event-handlers console_direct+
colcon test-result --test-result-base "$ROOT/build/packages" --verbose
