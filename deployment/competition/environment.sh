#!/usr/bin/env bash
# Source from this deployment; never borrow another project's hardware_ws.
COMPETITION_ROOT="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
export COMPETITION_ROOT
source /opt/ros/noetic/setup.bash
source "$COMPETITION_ROOT/vision_ws/devel/setup.bash"
source "$COMPETITION_ROOT/patrol_uav_ws-patrol_planner/devel/setup.bash"
for package in uav_mission uav_vision uav_high_view camera_sdk actuator_pwm; do
  resolved_package="$(rospack find "$package")" || return 1
  case "$resolved_package" in
    "$COMPETITION_ROOT"/*) ;;
    *) echo "Wrong overlay: $package resolved to $resolved_package" >&2; return 1 ;;
  esac
done
