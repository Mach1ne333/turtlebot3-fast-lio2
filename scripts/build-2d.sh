#!/usr/bin/env bash
# No FAST_LIO/ICP build needed. ROS setup is not nounset-safe.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/humble-base-env.bash"
# System resources used by tb3_sim; these are not local source/build targets.
ros2 pkg prefix turtlebot3_gazebo >/dev/null
ros2 pkg prefix turtlebot3_description >/dev/null
(cd "$TB3_SIM_WS" && colcon build --packages-select tb3_sim \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF)
source "$TB3_SIM_WS/install/local_setup.bash"
(cd "$TB3_CARTOGRAPHER_WS" && colcon build --packages-select cartographer_bringup \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=ON)
source "$TB3_CARTOGRAPHER_WS/install/local_setup.bash"
(cd "$TB3_NAV_WS" && CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select fast_planner fast_planner_nav2 \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=ON)
source "$TB3_NAV_WS/install/local_setup.bash"
for package in fast_planner fast_planner_nav2 tb3_sim cartographer_bringup; do
  prefix="$(ros2 pkg prefix "$package")"
  [[ "$prefix" == "$TB3_PROJECT_ROOT/"* ]] || { echo "Foreign package: $package -> $prefix" >&2; exit 1; }
  echo "$package -> $prefix"
done
