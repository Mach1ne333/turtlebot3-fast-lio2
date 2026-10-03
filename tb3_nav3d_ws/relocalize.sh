#!/usr/bin/env bash
# Initial body pose in the saved PCD map frame (meters and radians).
set -e
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"
MAP="${PCD_MAP:-$(dirname "$WS")/fast_lio2_ws/maps/tb3_fast_lio2.pcd}"
test -s "$MAP" || { echo "PCD map missing: $MAP" >&2; exit 1; }
x="${1:-0}"; y="${2:-0}"; z="${3:-0}"; yaw="${4:-0}"
reply=$(timeout 15 ros2 service call /pcd_localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$MAP', x: $x, y: $y, z: $z, yaw: $yaw, pitch: 0.0, roll: 0.0}")
printf '%s\n' "$reply"
echo "$reply" | grep -q 'success=True' || { echo 'PCD load failed' >&2; exit 1; }
echo "PCD loaded; waiting for actual ICP alignment (not just the load response)..."
for n in $(seq 1 12); do
  result=$(timeout 8 ros2 service call /pcd_localizer/relocalize_check interface/srv/IsValid '{code: 0}')
  if echo "$result" | grep -q 'valid=True'; then
    echo "3D PCD ICP alignment valid (attempt $n)."
    exit 0
  fi
  sleep 1
done
echo "ICP not valid. Check initial pose, PCD coverage, and live /cloud_registered_body." >&2
exit 1
