#!/usr/bin/env bash
# One-terminal FAST-LIO2 + 3D PCD relocalization + delayed Nav2 startup.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/../scripts/humble-base-env.bash"
# Load current prefixes explicitly, not the old underlay chain in setup.bash.
source "$TB3_SIM_WS/install/local_setup.bash"
source "$TB3_LIO_WS/install/local_setup.bash"
source "$WS/install/local_setup.bash"
PCD_MAP="$TB3_LIO_MAP"
MAP="$WS/maps/tb3_lio_ground.yaml"
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    pcd_map:=*) PCD_MAP="${arg#pcd_map:=}" ;;
    map:=*) MAP="${arg#map:=}" ;;
    *) EXTRA_ARGS+=("$arg") ;;
  esac
done
PCD_MAP="$(realpath -m -- "$PCD_MAP")"
MAP="$(realpath -m -- "$MAP")"
for file in "$PCD_MAP" "$MAP"; do
  test -f "$file" && test -s "$file" || { echo "Required map missing: $file" >&2; exit 1; }
done
if pgrep -x fastlio_mapping >/dev/null || pgrep -x localizer_node >/dev/null || pgrep -x bt_navigator >/dev/null; then
  echo "FAST-LIO2, PCD localizer, or Nav2 already running. Stop those launch terminals first (avoid duplicate TF/nodes)." >&2
  exit 1
fi
cd "$WS"  # Navigation runs from its own workspace, not the mapping workspace.
exec ros2 launch "$WS/launch/navigation_all.launch.py" \
  pcd_map:="$PCD_MAP" map:="$MAP" "${EXTRA_ARGS[@]}"
