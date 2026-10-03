#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
JOBS="${BUILD_JOBS:-2}"
OPENCV_ARGS=()
if [[ -n "${OPENCV_CMAKE_DIR:-}" ]]; then
  OPENCV_ARGS=("-DOpenCV_DIR=$OPENCV_CMAKE_DIR")
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
cd "$PROJECT_ROOT/vision_ws"
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES= "${OPENCV_ARGS[@]}" -j"$JOBS"
set +u
source "$PROJECT_ROOT/vision_ws/devel/setup.bash"
set -u
cd "$PROJECT_ROOT/patrol_uav_ws-patrol_planner"
catkin_make -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES= "${OPENCV_ARGS[@]}" -j"$JOBS"
