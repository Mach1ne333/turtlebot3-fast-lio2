#!/usr/bin/env bash
# Separate from LIO/ICP. Start start-world.sh 2d first; create your map yourself.
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/humble-2d-env.bash"
if pgrep -x fastlio_mapping >/dev/null || pgrep -x localizer_node >/dev/null || \
   pgrep -x bt_navigator >/dev/null || \
   pgrep -f '^/opt/ros/humble/lib/(slam_toolbox/[^ ]+|cartographer_ros/cartographer_node)' >/dev/null; then
  echo "SLAM, LIO, ICP or Nav2 is already running. Stop it before starting 2D AMCL navigation." >&2
  exit 1
fi
exec ros2 launch "$WS/launch/navigation_2d.launch.py" "$@"
