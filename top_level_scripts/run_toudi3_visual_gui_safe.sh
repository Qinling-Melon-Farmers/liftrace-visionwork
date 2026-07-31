#!/usr/bin/env bash
# Pure visual toudi3 GUI for manual inspection.
# No PX4, MAVROS, patrol_control, arming or actuator is launched.
set -euo pipefail

script_dir="${BASH_SOURCE[0]%/*}"
project_root="${script_dir%/*}"

# WSLg variables are not always inherited by non-interactive wsl -e shells.
if [[ -d /mnt/wslg/runtime-dir ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
  export PULSE_SERVER="${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
fi

export ROS_MASTER_URI="${VISUAL_GUI_ROS_MASTER_URI:-http://127.0.0.1:13331}"
export GAZEBO_MASTER_URI="${VISUAL_GUI_GAZEBO_MASTER_URI:-http://127.0.0.1:13345}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME || true
export ROS_LOG_DIR="${VISUAL_GUI_ROS_LOG_DIR:-/tmp/uav_vision_gui_roslog}"
mkdir -p "${ROS_LOG_DIR}"

# shellcheck disable=SC1091
source "${script_dir}/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

# Default: the 135-degree tent pressure pose that currently exposes
# tent/panzer ambiguity. Override through environment variables if needed.
camera_x="${VISUAL_GUI_CAMERA_X:-0.93558}"
camera_y="${VISUAL_GUI_CAMERA_Y:-0.17625}"
camera_z="${VISUAL_GUI_CAMERA_Z:-2.4}"
camera_yaw="${VISUAL_GUI_CAMERA_YAW:-2.356194}"
evaluation_seed="${VISUAL_GUI_SEED:-16}"

cd "${project_root}"
exec roslaunch uav_vision_eval toudi3_static_eval.launch \
  gui:=true \
  start_vision:=true \
  enable_debug_image:=true \
  record_metrics:=false \
  camera_x:="${camera_x}" \
  camera_y:="${camera_y}" \
  camera_z:="${camera_z}" \
  camera_yaw:="${camera_yaw}" \
  evaluation_seed:="${evaluation_seed}"
