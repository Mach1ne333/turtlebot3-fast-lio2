#!/usr/bin/env bash
# Initialize pinned FAST_LIO sources and apply this project's algorithm delta.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/workspaces.bash"
ROOT="$TB3_PROJECT_ROOT"
cd "$ROOT"
git submodule update --init --recursive
LIO="$TB3_LIO_WS/src/FAST_LIO"
PIN=a4743b095409588842a5b30ddfa27e29d2f99164
[ "$(git -C "$LIO" rev-parse HEAD)" = "$PIN" ] || {
  echo "FAST_LIO submodule is not at tested commit $PIN" >&2; exit 1;
}
patch="$TB3_LIO_WS/patches/fast_lio2_gazebo.patch"
if git -C "$LIO" apply --reverse --check "$patch" 2>/dev/null; then
  echo 'FAST_LIO patch already applied'
else
  git -C "$LIO" apply --check "$patch"
  git -C "$LIO" apply "$patch"
fi
# Integration config/RViz now belong to fast_lio_bringup, not upstream sources.

# tb3_sim is the local simulation package. Its mesh resources (and optional
# stock 2D model/world) come from system Humble packages, not source checkouts.
printf '\nSimulation resources must be installed separately, for example:\n'
printf '  sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description\n'
printf 'No official TurtleBot3 source checkouts are created by this script.\n'
printf '\nFAST_LIO sources ready. Next: see README build instructions.\n'
printf 'FAST_LIO submodule is expected to appear modified locally after applying the patch.\n'
