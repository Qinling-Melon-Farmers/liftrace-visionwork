#!/usr/bin/env bash
# WSLg image viewer for the safe visual-only GUI session.
set -eo pipefail

project_root="${PROJECT_ROOT:-/home/xhj/liftrace}"
if [[ -d /mnt/wslg/runtime-dir ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
  export PULSE_SERVER="${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
fi

export ROS_MASTER_URI="${VISUAL_GUI_ROS_MASTER_URI:-http://127.0.0.1:13331}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME || true

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${project_root}/vision_ws/devel/setup.bash"
set -u

image_topic="${VISUAL_GUI_IMAGE_TOPIC:-/downward_camera/image_raw}"
exec rosrun rqt_image_view rqt_image_view "${image_topic}"
