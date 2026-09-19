#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
project_root="$(cd "$script_dir/../../../../.." && pwd)"
trial="${1:-}"
mode="${2:-}"
if [[ "$mode" != preview && "$mode" != flight ]]; then
  echo "Usage: start.sh preview|flight [--model /path/model.rknn]" >&2
  exit 2
fi
shift 2
set +u
source /opt/ros/noetic/setup.bash
source "$project_root/vision_ws/devel/setup.bash"
source "$project_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
set -u
for package in uav_mission uav_vision uav_high_view uav_board_trials; do
  actual="$(readlink -f "$(rospack find "$package")")"
  case "$package" in
    uav_mission) expected="$project_root/patrol_uav_ws-patrol_planner/src/$package" ;;
    uav_board_trials) expected="$project_root/deployment/board_trials_4x4/common/$package" ;;
    *) expected="$project_root/vision_ws/src/$package" ;;
  esac
  [[ "$actual" == "$(readlink -f "$expected")" ]] || { echo "Wrong package overlay: $package -> $actual" >&2; exit 2; }
done
exec "${BOARD_PYTHON:-/usr/bin/python3}" "$script_dir/run_trial.py" "$trial" "$mode" --root "$project_root" "$@"
