#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
project_root="$(cd "$script_dir/../.." && pwd)"
set +u
source /opt/ros/noetic/setup.bash
source "$project_root/vision_ws/devel/setup.bash"
source "$project_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
set -u
exec roslaunch camera_sdk camera_calibrated_1280x720.launch "video_devices:=${1:-/dev/video0}"
