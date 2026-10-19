#!/usr/bin/env bash
set -euo pipefail
script_path="${BASH_SOURCE[0]}"
script_parent="${script_path%/*}"
[[ "$script_parent" != "$script_path" ]] || script_parent=.
script_dir="$(cd -- "$script_parent" && pwd)"
project_root="/home/orangepi/liftrace_r64_onboard_405bda42"
mode="${1:-help}"
if [[ "$mode" == help || "$mode" == --help ]]; then
  printf '%s\n' 'Usage: bash start_test.sh check|preview|flight [start_camera:=true] [video_devices:=/dev/video0]' 'check: statically verify preview and flight configurations; no nodes.' 'preview: mapping and vision only.' 'flight: control and real servo startup; requires separate operator authorization.' 'Fixed ground Z=-0.25, cruise Z=0.25, AGL=0.50 m.'
  exit 0
fi
if [[ "$mode" != check && "$mode" != preview && "$mode" != flight ]]; then
  printf '%s\n' 'Unknown mode.' >&2
  exit 2
fi
shift
for argument in "$@"; do
  case "$argument" in
    start_camera:=true|start_camera:=false|video_devices:=/dev/*) ;;
    *) printf 'Unsupported override: %s\n' "$argument" >&2; exit 2 ;;
  esac
done
set +u
source /opt/ros/noetic/setup.bash
source "$project_root/vision_ws/devel/setup.bash"
source "$project_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_PACKAGE_PATH="$script_dir:${ROS_PACKAGE_PATH:-}"
if [[ "$mode" == check ]]; then
  /usr/bin/python3 "$script_dir/tests/validate_launch.py"
  exit 0
fi
nodes="$(rosnode list)" || { printf '%s\n' 'Start the existing device-side ROS/MAVROS/Livox chain first.' >&2; exit 2; }
for node in /laserMapping /freedom /patrol_control /fast_planner_node /navigation_frame_adapter /target_detector_rknn /cross_detector /circle_detector /landing_detector /detection_fusion /target_refiner /target_map_projector /target_memory /drop_aligner /navigation/mission_manager /navigation/planner_bridge /servo_controller1 /guarded_servo_proxy /right_servo_once; do
  if [[ $'\n'"$nodes"$'\n' == *$'\n'"$node"$'\n'* ]]; then
    printf 'Existing node %s; this entry does not stop other applications.\n' "$node" >&2
    exit 2
  fi
done
if [[ "$mode" == preview ]]; then
  exec roslaunch "$script_dir/test.launch" "$@" enable_control_output:=false start_servo:=false
fi
if [[ "${HARDWARE_TEST_AUTHORIZED:-0}" != 1 ]]; then
  printf '%s\n' 'flight starts real servo hardware (all three servos reset on startup). Set HARDWARE_TEST_AUTHORIZED=1 only after explicit on-site authorization.' >&2
  exit 2
fi
for chip in 4 5 0; do
  if [[ ! -w "/sys/class/pwm/pwmchip$chip/pwm0/duty_cycle" ]]; then
    printf 'PWM chip %s is not prepared. The existing node resets all three servos; initialize hardware separately. This script never runs sudo or initializes PWM.\n' "$chip" >&2
    exit 2
  fi
done
mkdir -p "$script_dir/runs"
run_dir="$(mktemp -d "$script_dir/runs/session_XXXXXX")"
export ROS_LOG_DIR="$run_dir/roslog"
printf 'Session: %s\n' "$run_dir"
exec roslaunch "$script_dir/test.launch" "$@" enable_control_output:=true start_servo:=true "receipt_path:=$run_dir/right_release.receipt"
