#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=burger
# Default: the official 2D Burger. Select the 3D point-cloud view with "3d".
MODE="${1:-2d}"
case "$MODE" in
  2d)
    CONFIG="$WS/src/turtlebot3_simulations/turtlebot3_gazebo/rviz/tb3_gazebo.rviz"
    ;;
  3d)
    CONFIG="$WS/src/turtlebot3_lio_sim/rviz/tb3_lidar3d.rviz"
    ;;
  *)
    echo "Usage: $0 [2d|3d]" >&2
    exit 2
    ;;
esac
exec rviz2 -d "$CONFIG"
