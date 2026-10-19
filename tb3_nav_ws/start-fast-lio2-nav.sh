#!/usr/bin/env bash
# Debug only: navigation LIO odometry, without ICP/Nav2 or map saving.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/workspaces.bash"
source /opt/ros/humble/setup.bash
source "$TB3_LIO_WS/install/setup.bash"  # Reuse the compiled fast_lio algorithm.
source "$WS/install/setup.bash"
cd "$WS"
exec ros2 launch "$WS/launch/lio_odometry.launch.py" "$@"
