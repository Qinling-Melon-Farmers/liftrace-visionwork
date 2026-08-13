#!/usr/bin/env bash

# Laptop-only full control-flow SITL viewer:
# uav_vision -> legacy compatibility topics -> patrol_control/planner ->
# PX4/MAVROS -> Gazebo aircraft response.
#
# FAST-LIO/FreeDOM and RViz are disabled by default to keep WSL SITL real-time
# enough for PX4 preflight checks. No actuator_pwm or hardware endpoint is
# started by the included simulation launch.
set -e

project_root="${PROJECT_ROOT:-/home/xhj/liftrace}"
script_dir="${BASH_SOURCE[0]%/*}"

# shellcheck disable=SC1091
source "${script_dir}/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

exec "${script_dir}/sim_run.sh" toudi4_control_flow_gui \
  roslaunch patrol_control toudi3_full_competition_sim_new_vision.launch \
  world:="${TOUDI3_WORLD}" \
  px4_root:="${PX4_ROOT}" \
  astra_sim_lib:="${ASTRA_SIM_LIB}" \
  px4_plugin_lib:="${PX4_PLUGIN_LIB}" \
  vision_python:="${VISION_PYTHON}" \
  gui:=true \
  rviz:="${TOUDI3_CONTROL_RVIZ:-false}" \
  mapping_rviz:=false \
  start_mapping:="${TOUDI3_CONTROL_MAPPING:-false}" \
  start_arming:=true \
  waypoint_mode:=true \
  simulation_auto_land:=true \
  px4_max_distance:=0.2 \
  enable_debug_image:=true
