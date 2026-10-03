#!/usr/bin/env bash
# One-terminal FAST-LIO2 + 3D PCD relocalization + delayed Nav2 startup.
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$WS")"
source "$ROOT/turtlebot3_ws/install/setup.bash"
source "$ROOT/fast_lio2_ws/install/setup.bash"
source "$WS/install/setup.bash"
for file in "$ROOT/fast_lio2_ws/maps/tb3_fast_lio2.pcd" \
            "$WS/maps/tb3_lio_ground.yaml"; do
  test -s "$file" || { echo "Required map missing: $file" >&2; exit 1; }
done
if pgrep -x fastlio_mapping >/dev/null || pgrep -x localizer_node >/dev/null || pgrep -x bt_navigator >/dev/null; then
  echo "FAST-LIO2, PCD localizer, or Nav2 already running. Stop those launch terminals first (avoid duplicate TF/nodes)." >&2
  exit 1
fi
cd "$ROOT/fast_lio2_ws"  # keep any relative LIO map paths workspace-local
exec ros2 launch "$WS/launch/navigation_all.launch.py"
