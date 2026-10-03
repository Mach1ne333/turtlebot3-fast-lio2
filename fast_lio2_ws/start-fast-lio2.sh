#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
mkdir -p "$WS/maps"
cd "$WS"  # YAML map_file_path is relative to this workspace
exec ros2 launch fast_lio mapping.launch.py \
  config_file:=tb3_gazebo_instant.yaml use_sim_time:=true rviz:=false "$@"
