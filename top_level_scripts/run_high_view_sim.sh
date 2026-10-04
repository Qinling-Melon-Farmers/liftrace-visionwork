#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
project_root="$(cd "$script_dir/.." && pwd)"
seed="${HIGH_VIEW_SEED:-31}"
route="${HIGH_VIEW_ROUTE:-rectangle}"
while (($#)); do
  case "$1" in
    --seed) seed="${2:?Missing seed}"; shift 2;;
    --route) route="${2:?Missing route}"; shift 2;;
    *) echo "Usage: run_high_view_sim.sh [--seed N] [--route rectangle|snake2|snake3|rectangle_baseline]" >&2; exit 2;;
  esac
done
[[ "${SIM_RUN_AUTHORIZED:-0}" == 1 ]] || { echo "A current explicit simulation request is required" >&2; exit 2; }
[[ "$seed" =~ ^[0-9]+$ ]] || { echo "Invalid seed" >&2; exit 2; }
case "$route" in rectangle|snake2|snake3|rectangle_baseline) ;; *) echo "Invalid route" >&2; exit 2;; esac
: "${UAV_VISION_MODEL_PATH:?Set UAV_VISION_MODEL_PATH to the five-class weights}"
export UAV_WS="$project_root/patrol_uav_ws-patrol_planner"
export VISION_WS="$project_root/vision_ws"
export ASTRA_MODEL_ROOT="$project_root/simulation_assets/models"
export GAZEBO_MODEL_PATH="$project_root/vision_ws/src/uav_vision_eval/models:$ASTRA_MODEL_ROOT${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export SIM_REQUIRE_GATE=1
scene_base="$project_root/logs/generated/high_view_${route}_${seed}_$(date +%Y%m%d_%H%M%S)"
"${SCENE_PYTHON:-/usr/bin/python3}" "$project_root/simulation_tools/prepare_high_view_scene.py" --seed "$seed" --route "$route" --output "$scene_base"
exec "$script_dir/sim_run.sh" "high_view_${route}_seed${seed}" \
 bash "$script_dir/roslaunch_rl_drone.sh" "$project_root/docs/verification/route_speed_20261004/replay.launch" \
 "scene_dir:=$scene_base/${route}_seed${seed}" "field_seed:=$seed" "target_model_path:=$UAV_VISION_MODEL_PATH"
