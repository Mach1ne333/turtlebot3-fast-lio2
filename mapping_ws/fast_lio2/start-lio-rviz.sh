#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
# RViz configuration selects /Laser_map_display; launch its publisher as well.
# Close RViz (or Ctrl+C) to stop the display filter, without stopping FAST-LIO.
exec ros2 launch fast_lio_bringup lio_rviz.launch.py "$@"
