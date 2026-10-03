#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$(dirname "$WS")/turtlebot3_ws/install/setup.bash"  # robot meshes
source "$WS/install/setup.bash"
exec rviz2 -d "$WS/rviz/nav3d.rviz" --ros-args -p use_sim_time:=true
