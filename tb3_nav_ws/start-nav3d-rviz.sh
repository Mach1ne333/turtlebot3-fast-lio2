#!/usr/bin/env bash
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/humble-base-env.bash"
source "$TB3_SIM_WS/install/local_setup.bash"  # robot meshes
source "$WS/install/local_setup.bash"
exec rviz2 -d "$WS/rviz/nav3d.rviz" --ros-args -p use_sim_time:=true
