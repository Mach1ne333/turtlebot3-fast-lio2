#!/usr/bin/env bash
# Refuse to run Nav2 without an actually successful 3D PCD alignment.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/humble-base-env.bash"
source "$WS/install/local_setup.bash"
MAP="$WS/maps/tb3_lio_ground.yaml"
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    map:=*) MAP="${arg#map:=}" ;;
    *) EXTRA_ARGS+=("$arg") ;;
  esac
done
MAP="$(realpath -m -- "$MAP")"
test -f "$MAP" && test -s "$MAP" || { echo "Missing PCD-derived planning map: $MAP" >&2; exit 1; }
if ! timeout 12 ros2 service call /pcd_localizer/relocalize_check interface/srv/IsValid '{code: 0}' | grep -q 'valid=True'; then
  echo "3D PCD localization not valid: do not start Nav2. Run relocalize.sh and check ICP." >&2
  exit 1
fi
if ! timeout 12 python3 "$WS/tools/check_nav_freshness.py"; then
  echo "LIO / map TF is stale; Nav2 cannot safely use this localization." >&2
  exit 1
fi
exec ros2 launch "$WS/launch/nav3d.launch.py" map:="$MAP" "${EXTRA_ARGS[@]}"
