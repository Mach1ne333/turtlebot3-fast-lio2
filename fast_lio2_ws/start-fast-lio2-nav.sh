#!/usr/bin/env bash
# Localization against an existing PCD; disables growing /Laser_map and /path.
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
cd "$WS"
exec ros2 launch fast_lio mapping.launch.py \
  config_file:=tb3_gazebo_nav.yaml use_sim_time:=true rviz:=false "$@"
