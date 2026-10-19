#!/usr/bin/env bash
# Source in a dedicated terminal. Start from system Humble, not stale overlays.
source "$(dirname "${BASH_SOURCE[0]}")/workspaces.bash"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:/usr/local/sbin
source /opt/ros/humble/setup.bash
