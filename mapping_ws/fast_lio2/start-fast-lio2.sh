#!/usr/bin/env bash
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../../scripts/workspaces.bash"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$(dirname "$TB3_LIO_MAP")"
cd "$WS"  # Upstream logs remain workspace-local; map path is absolute.
exec ros2 launch fast_lio_bringup mapping.launch.py \
  config_file:=tb3_gazebo_instant.yaml map_file_path:="$TB3_LIO_MAP" \
  use_sim_time:=true rviz:=false "$@"
