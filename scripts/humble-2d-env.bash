#!/usr/bin/env bash
# Source in a dedicated terminal: system Humble + this repository, never old overlays.
source "$(dirname "${BASH_SOURCE[0]}")/humble-base-env.bash"
for _tb3_ws in "$TB3_SIM_WS" "$TB3_CARTOGRAPHER_WS" "$TB3_NAV_WS"; do
  if [[ -f "$_tb3_ws/install/local_setup.bash" ]]; then
    source "$_tb3_ws/install/local_setup.bash"
  fi
done
export TURTLEBOT3_MODEL=burger
unset _tb3_ws
