#!/usr/bin/env bash

# GUI SITL entry point using the new uav_vision Phase-D chain.
# Safe simulation only: no actuator_pwm and no hardware MAVROS endpoint.
set -e

PROJECT_ROOT="${PROJECT_ROOT:-/home/xhj/liftrace}"
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

exec roslaunch patrol_control toudi3_full_competition_sim_new_vision.launch \
  world:="${TOUDI3_WORLD}" \
  px4_root:="${PX4_ROOT}" \
  astra_sim_lib:="${ASTRA_SIM_LIB}" \
  px4_plugin_lib:="${PX4_PLUGIN_LIB}" \
  vision_python:="${VISION_PYTHON}" \
  gui:=true rviz:=true mapping_rviz:=false \
  start_mapping:=true start_arming:=true \
  waypoint_mode:=true simulation_auto_land:=true px4_max_distance:=0.2
