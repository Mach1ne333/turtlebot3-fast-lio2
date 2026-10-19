#!/usr/bin/env bash
# Source this file; keep cross-workspace directory names in one place.
# Resolve from this file, not the caller's working directory or a user's home.
TB3_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TB3_SIM_WS="$TB3_PROJECT_ROOT/turtlebot3_ws"
TB3_CARTOGRAPHER_WS="$TB3_PROJECT_ROOT/mapping_ws/cartographer"
TB3_LIO_WS="$TB3_PROJECT_ROOT/mapping_ws/fast_lio2"
TB3_LIO_MAP="$TB3_LIO_WS/maps/tb3_fast_lio2.pcd"
TB3_NAV_WS="$TB3_PROJECT_ROOT/tb3_nav_ws"
