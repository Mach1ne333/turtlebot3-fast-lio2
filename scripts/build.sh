#!/usr/bin/env bash
# Compile the three isolated workspaces in their tested overlay order.
# ROS setup.bash is not nounset-safe on Humble.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
for name in turtlebot3_ws fast_lio2_ws tb3_nav3d_ws; do
  ws="$ROOT/$name"
  ( cd "$ws" && CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
      --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF )
  source "$ws/install/setup.bash"
done
printf '\nBuilt all three workspaces. Follow README to launch Gazebo/LIO/RViz.\n'
