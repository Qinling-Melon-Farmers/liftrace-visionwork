#!/usr/bin/env bash
set -e
task_dir=$(cd "${BASH_SOURCE[0]%/*}" && pwd)
task_root=$(cd "$task_dir/../../../.." && pwd)
mode="${1:?baseline or strategy required}"
case "$mode" in baseline) strategy=false;; strategy) strategy=true;; *) exit 64;; esac
cd "$task_root"
export ASTRA_MODEL_ROOT="$task_root/simulation_assets/models"
export GAZEBO_MODEL_PATH="$task_root/vision_ws/src/uav_vision_eval/models:$ASTRA_MODEL_ROOT"
export SIM_STORAGE_GUARD_PATH="${SIM_STORAGE_GUARD_PATH:-/mnt/f}" SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1
exec timeout --signal=TERM --kill-after=30s 2850s bash top_level_scripts/sim_run.sh \
  "fixed_fast_${mode}_seed34" bash top_level_scripts/roslaunch_rl_drone.sh \
  uav_high_view fast_comparison.launch "strategy:=$strategy" field_seed:=34 \
  "target_model_path:=${UAV_VISION_MODEL_PATH:-/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt}" \
  "world:=$task_dir/seed34/field.world" "field_config:=$task_dir/seed34/field_config.yaml" \
  "runtime_config:=$task_dir/seed34/runtime.yaml" "gate_geometry_config:=$task_dir/seed34/gate_geometry.yaml"
