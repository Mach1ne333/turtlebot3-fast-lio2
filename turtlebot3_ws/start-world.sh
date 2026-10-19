#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/humble-2d-env.bash"
export GAZEBO_MODEL_DATABASE_URI=""
cd "$WS"

# Keep positional model selection; scene defaults are declared in tb3_sim.launch.py.
MODE=2d
if [[ $# -gt 0 && "$1" != *:=* && "$1" != --* ]]; then
  MODE="$1"
  shift
fi
case "$MODE" in
  2d|3d|3d-light|3d-nav) ;;
  *) echo "Usage: $0 [2d|3d|3d-light|3d-nav] [scene:=shiyanlou_third|shiyanlou|turtlebot3_world] [ROS launch arguments]" >&2; exit 2 ;;
esac
if pgrep -x gzserver >/dev/null; then
  echo "Gazebo is already running. Stop the previous world before switching to $MODE." >&2
  exit 1
fi
exec ros2 launch tb3_sim tb3_sim.launch.py robot_mode:="$MODE" "$@"
