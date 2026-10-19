#!/usr/bin/env bash
# Mapping only. Start start-world.sh 2d first; never run AMCL/another SLAM too.
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../../scripts/humble-2d-env.bash"
if pgrep -f '^/opt/ros/humble/lib/(slam_toolbox/[^ ]+|cartographer_ros/cartographer_node|nav2_amcl/amcl|nav2_bt_navigator/bt_navigator)' >/dev/null || \
   pgrep -x fastlio_mapping >/dev/null || pgrep -x localizer_node >/dev/null; then
  echo "SLAM, AMCL/Nav2 or LIO localization is already running. Stop it before starting Cartographer." >&2
  exit 1
fi
exec ros2 launch cartographer_bringup cartographer_2d.launch.py "$@"
