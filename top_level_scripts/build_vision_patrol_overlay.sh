#!/usr/bin/env bash

# Build the vision workspace first, then build the patrol workspace with
# vision_ws as its catkin underlay.  No ROS nodes or hardware are started.
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/home/xhj/liftrace}"
VISION_WS="${VISION_WS:-${PROJECT_ROOT}/vision_ws}"
UAV_WS="${UAV_WS:-${PROJECT_ROOT}/patrol_uav_ws-patrol_planner}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "missing /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u
cd "${VISION_WS}"
catkin_make -j1

set +u
# shellcheck disable=SC1090
source "${VISION_WS}/devel/setup.bash"
set -u
cd "${UAV_WS}"
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES="" -j1

