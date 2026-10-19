#!/usr/bin/env bash
# Compile simulation, mapping integrations, LIO and navigation in overlay order.
# ROS setup.bash is not nounset-safe on Humble.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/humble-base-env.bash"
for ws in "$TB3_SIM_WS" "$TB3_CARTOGRAPHER_WS" "$TB3_LIO_WS" "$TB3_NAV_WS"; do
  packages=()
  if [[ "$ws" == "$TB3_SIM_WS" ]]; then
    # Do not rebuild old official source checkouts, even if still present.
    packages=(--packages-select tb3_sim)
  fi
  ( cd "$ws" && CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build "${packages[@]}" \
      --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF )
  source "$ws/install/local_setup.bash"
done
printf '\nBuilt all four workspaces. Follow README to launch Gazebo/LIO/RViz.\n'
