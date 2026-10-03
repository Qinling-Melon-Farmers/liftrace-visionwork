#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
JOBS="${BUILD_JOBS:-2}"
source /opt/ros/noetic/setup.bash
OPENCV_ARGS=()
if [[ -n "${OPENCV_CMAKE_DIR:-}" ]]; then OPENCV_ARGS=("-DOpenCV_DIR=$OPENCV_CMAKE_DIR"); fi
cd "$ROOT/vision_ws"
catkin_make --only-pkg-with-deps uav_vision uav_high_view camera_sdk \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 "${OPENCV_ARGS[@]}" -j"$JOBS"
source "$ROOT/vision_ws/devel/setup.bash"
cd "$ROOT/patrol_uav_ws-patrol_planner"
catkin_make --only-pkg-with-deps uav_mission actuator_pwm fast_lio freedom livox_ros_driver2 \
  -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 "${OPENCV_ARGS[@]}" -j"$JOBS"
