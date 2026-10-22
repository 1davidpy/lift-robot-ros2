#!/bin/bash
set -eo pipefail
cd /home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
source /opt/ros/humble/setup.bash
source build/install/setup.bash
cmake --build build/packages/luxi_device_adapters --target r680_adapter -j2
cmake -S /home/sunrise/.local/share/luxi-atlas-carrier/sdk-relay -B .local/atlas-navigation-20261022/relay-build -DCMAKE_BUILD_TYPE=Release
cmake --build .local/atlas-navigation-20261022/relay-build -j2
