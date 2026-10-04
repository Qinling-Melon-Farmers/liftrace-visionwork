#!/usr/bin/env bash
# Called ONLY inside sim_run.sh; not a hardware entry.
set -euo pipefail
: "${SIM_RUN_DIR:?Invoke through top_level_scripts/sim_run.sh}"
script_dir="${BASH_SOURCE[0]%/*}"
project_root="$(cd "$script_dir/../../../../.." && pwd)"
trial="${1:?trial required}"
model="${2:?YOLO model path required}"
python="${VISION_PYTHON:-/home/xhj/miniconda3/envs/rl_drone/bin/python}"
"$python" "$script_dir/prepare_simulation.py" "$trial" "$SIM_RUN_DIR/generated"
mode="$trial"
case "$trial" in corridor_landing) mode=landing ;; full_mission) mode=high_view_full ;; esac
exec bash "$project_root/top_level_scripts/roslaunch_rl_drone.sh" uav_board_trials simulation.launch \
  "generated_dir:=$SIM_RUN_DIR/generated" "trial:=$trial" "mode:=$mode" "model_path:=$model"
