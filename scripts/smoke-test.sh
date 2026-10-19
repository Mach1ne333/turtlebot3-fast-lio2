#!/usr/bin/env bash
# Isolated headless integration test; run only when no other Gazebo is active.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/humble-base-env.bash"
source "$TB3_SIM_WS/install/local_setup.bash"
source "$TB3_LIO_WS/install/local_setup.bash"
source "$TB3_NAV_WS/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11445}"
export GAZEBO_MODEL_DATABASE_URI=""
LOGDIR="${1:-$(mktemp -d /tmp/tb3-fastlio2.XXXXXX)}"
mkdir -p "$LOGDIR"
echo "Isolated test logs: $LOGDIR"
world=""; nav=""; observer=""
cleanup() {
  for p in "$observer" "$nav" "$world"; do
    if [ -n "$p" ]; then kill -TERM -- "-$p" 2>/dev/null || :; fi
  done
  sleep 3
  # Gazebo may detach from the launch process group; match BOTH test env keys.
  for proc in /proc/[0-9]*; do
    pid=${proc##*/}
    [ "$pid" = "$$" ] && continue
    [ -r "$proc/environ" ] || continue
    if tr '\0' '\n' < "$proc/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" &&
       tr '\0' '\n' < "$proc/environ" 2>/dev/null | grep -qx "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"; then
      kill -TERM "$pid" 2>/dev/null || :
    fi
  done
}
trap cleanup EXIT
# The bundled PCD/grid match the system-installed official world, not the
# launcher's current default building. Keep this test's scene/map pair fixed.
setsid "$TB3_SIM_WS/start-world.sh" 3d-nav scene:=turtlebot3_world gui:=false > "$LOGDIR/world.log" 2>&1 & world=$!
for n in $(seq 1 35); do
  if timeout 5 ros2 topic list | grep -qx /points_raw; then break; fi
  sleep 2
done
timeout 6 ros2 topic list | grep -qx /points_raw || { echo 'No 3D point cloud'; exit 1; }
setsid "$TB3_NAV_WS/start-navigation.sh" > "$LOGDIR/navigation.log" 2>&1 & nav=$!
for n in $(seq 1 35); do
  if timeout 6 ros2 service list | grep -qx /pcd_localizer/relocalize; then break; fi
  sleep 2
done
timeout 6 ros2 service list | grep -qx /pcd_localizer/relocalize || { echo 'No ICP service'; exit 1; }
if [ "$(timeout 6 ros2 node list 2>/dev/null | grep -cx /bt_navigator || :)" -gt 0 ]; then
  echo 'BUG: Nav2 started before 3D ICP alignment'; exit 1
fi
echo 'Nav2 correctly gated before RViz initial pose'
for n in $(seq 1 30); do
  if timeout 5 ros2 topic info /cloud_registered_body 2>/dev/null | grep -q 'Publisher count: 1'; then break; fi
  sleep 2
done
sleep 4
# Publish the same message that RViz 2D Pose Estimate emits at the mapping spawn.
timeout 15 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.032, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' > "$LOGDIR/initialpose.log" 2>&1
ready=0
for n in $(seq 1 45); do
  actions="$(timeout 7 ros2 action list -t 2>/dev/null || :)"
  if grep '^/navigate_to_pose ' <<< "$actions" >/dev/null; then ready=1; break; fi
  sleep 2
done
[ "$ready" = 1 ] || { echo 'Nav2 never started after valid ICP'; tail -35 "$LOGDIR/navigation.log"; exit 1; }
for name in map_server controller_server planner_server bt_navigator; do
  timeout 8 ros2 lifecycle get "/$name" | grep -q 'active \[3\]' || { echo "$name not active"; exit 1; }
done
echo '3D PCD ICP valid: Nav2 map/planner/controller/BT active'
# A goal result by itself is not proof of movement; compare actual odometry.
setsid python3 "$TB3_NAV_WS/tools/observe_motion.py" 120 > "$LOGDIR/motion.log" 2>&1 & observer=$!
sleep 4
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' > "$LOGDIR/goal.log" 2>&1
grep -q 'SUCCEEDED' "$LOGDIR/goal.log" || { echo 'Nav2 goal not successful'; tail -30 "$LOGDIR/goal.log"; exit 1; }
kill -INT -- "-$observer" 2>/dev/null || :  # observer flushes positions on KeyboardInterrupt
sleep 2
python3 - "$LOGDIR/motion.log" <<'PY'
import ast, math, pathlib, sys
text = pathlib.Path(sys.argv[1]).read_text().splitlines()
line = next((l for l in text if l.startswith('/Odometry messages=')), None)
if not line or 'start=' not in line or 'end=' not in line:
    raise SystemExit('No actual LIO movement samples')
a = ast.literal_eval(line.split('start=')[1].split(' end=')[0])
b = ast.literal_eval(line.split('end=')[1])
distance = math.dist(a, b)
print(f'Fast-LIO2 traveled {distance:.3f} m (must exceed 0.35 m)')
if distance < 0.35:
    raise SystemExit('Goal result without sufficient observed movement')
PY
echo 'PASS: PCD localization -> gated Nav2 -> goal + actual LIO movement'
