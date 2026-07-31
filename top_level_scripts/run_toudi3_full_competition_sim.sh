#!/usr/bin/env bash

# SITL-only launcher for the complete toudi3 competition chain.
# Do not use this script on a vehicle or with a hardware MAVROS endpoint.

set -e

# ROS Noetic helper scripts such as gazebo_ros/spawn_model use
# /usr/bin/env python3.  Keep the SITL launcher on the system ROS Python
# instead of inheriting a user's conda base interpreter, whose package set
# is not guaranteed to contain ROS dependencies such as PyYAML.
export PATH="/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH

PROJECT_ROOT="${PROJECT_ROOT:-/home/xhj/liftrace}"
UAV_WS="${UAV_WS:-${PROJECT_ROOT}/patrol_uav_ws-patrol_planner}"
PX4_ROOT="${PX4_ROOT:-/home/xhj/PX4-Autopilot}"
ASTRA_SIM_LIB="${ASTRA_SIM_LIB:-/home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib}"
PX4_PLUGIN_LIB="${PX4_PLUGIN_LIB:-${PX4_ROOT}/build/px4_sitl_default/build_gazebo-classic}"
WORLD="${TOUDI3_WORLD:-${UAV_WS}/toudi3.world}"
TOUDI3_GUI="${TOUDI3_GUI:-false}"
TOUDI3_RVIZ="${TOUDI3_RVIZ:-false}"
TOUDI3_MAPPING_RVIZ="${TOUDI3_MAPPING_RVIZ:-false}"
TOUDI3_START_VISUAL="${TOUDI3_START_VISUAL:-true}"
TOUDI3_START_MAPPING="${TOUDI3_START_MAPPING:-true}"
TOUDI3_START_ARMING="${TOUDI3_START_ARMING:-true}"
TOUDI3_WAYPOINT_MODE="${TOUDI3_WAYPOINT_MODE:-true}"
TOUDI3_SIMULATION_AUTO_LAND="${TOUDI3_SIMULATION_AUTO_LAND:-true}"
TOUDI3_PX4_MAX_DISTANCE="${TOUDI3_PX4_MAX_DISTANCE:-0.2}"

source /opt/ros/noetic/setup.bash
if [[ -f "${PROJECT_ROOT}/vision_ws/devel/setup.bash" ]]; then
  source "${PROJECT_ROOT}/vision_ws/devel/setup.bash"
fi
source "${UAV_WS}/devel/setup.bash"

export PX4_ROOT
export ROS_PACKAGE_PATH="/opt/ros/noetic/share:${PX4_ROOT}:${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic:${UAV_WS}/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="${ASTRA_SIM_LIB}:${PX4_PLUGIN_LIB}:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${ASTRA_SIM_LIB}:${PX4_PLUGIN_LIB}:${LD_LIBRARY_PATH:-}"

exec roslaunch patrol_control patrol_full_competition_sim.launch \
  world:="${WORLD}" \
  px4_root:="${PX4_ROOT}" \
  astra_sim_lib:="${ASTRA_SIM_LIB}" \
  px4_plugin_lib:="${PX4_PLUGIN_LIB}" \
  gui:="${TOUDI3_GUI}" \
  rviz:="${TOUDI3_RVIZ}" \
  mapping_rviz:="${TOUDI3_MAPPING_RVIZ}" \
  start_visual:="${TOUDI3_START_VISUAL}" \
  start_mapping:="${TOUDI3_START_MAPPING}" \
  start_arming:="${TOUDI3_START_ARMING}" \
  waypoint_mode:="${TOUDI3_WAYPOINT_MODE}" \
  simulation_auto_land:="${TOUDI3_SIMULATION_AUTO_LAND}" \
  px4_max_distance:="${TOUDI3_PX4_MAX_DISTANCE}"
