#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=burger
# Use local Gazebo models to avoid remote database startup timeouts.
export GAZEBO_MODEL_DATABASE_URI=""
cd "$WS"

# Default to the original 2D Burger; change "2d" below if you want a new default.
MODE="${1:-2d}"
if [ "$#" -gt 0 ]; then shift; fi
case "$MODE" in
  2d|3d|3d-light|3d-nav) ;;
  *) echo "Usage: $0 [2d|3d|3d-light|3d-nav] [optional ROS launch arguments]" >&2; exit 2 ;;
esac

# Both modes use the same Gazebo master. Stop the previous world before switching.
if pgrep -x gzserver >/dev/null; then
  echo "Gazebo is already running. Stop the previous world before switching to $MODE." >&2
  exit 1
fi

case "$MODE" in
  2d) exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py "$@" ;;
  3d) exec ros2 launch turtlebot3_lio_sim lidar3d_world.launch.py "$@" ;;
  3d-light)
    MODEL_DIR="$(ros2 pkg prefix turtlebot3_lio_sim)/share/turtlebot3_lio_sim/models/turtlebot3_burger_lidar3d"
    test -f "$MODEL_DIR/model_light.sdf" || { echo "Light model missing; rebuild turtlebot3_lio_sim" >&2; exit 1; }
    exec ros2 launch turtlebot3_lio_sim lidar3d_world.launch.py model_file:="$MODEL_DIR/model_light.sdf" "$@"
    ;;
  3d-nav)
    MODEL_DIR="$(ros2 pkg prefix turtlebot3_lio_sim)/share/turtlebot3_lio_sim/models/turtlebot3_burger_lidar3d"
    test -f "$MODEL_DIR/model_nav.sdf" || { echo "Nav model missing; rebuild turtlebot3_lio_sim" >&2; exit 1; }
    exec ros2 launch turtlebot3_lio_sim lidar3d_world.launch.py model_file:="$MODEL_DIR/model_nav.sdf" "$@"
    ;;
esac
