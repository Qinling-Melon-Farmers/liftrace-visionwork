#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
JOBS="${BUILD_JOBS:-2}"
# main retains historical source assets. Build exactly the competition packages.
set +u
source /opt/ros/noetic/setup.bash
set -u
cd "$PROJECT_ROOT/vision_ws"
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="camera_sdk;uav_vision;uav_vision_eval" -j"$JOBS"
set +u
source "$PROJECT_ROOT/vision_ws/devel/setup.bash"
set -u
cd "$PROJECT_ROOT/patrol_uav_ws-patrol_planner"
catkin_make -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="bspline;bspline_opt;catkin_simple;cmake_utils;cv_bridge;fast_lio;freedom;livox_ros_driver;path_searching;patrol_control;plan_env;plan_manage;poly_traj;pose_utils;quadrotor_msgs;traj_utils;uav_mission;uav_utils;waypoint_generator" -j"$JOBS"
