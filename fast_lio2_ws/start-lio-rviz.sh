#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
exec rviz2 -d "$WS/src/FAST_LIO/rviz/tb3_fast_lio2.rviz" \
  --ros-args -p use_sim_time:=true
