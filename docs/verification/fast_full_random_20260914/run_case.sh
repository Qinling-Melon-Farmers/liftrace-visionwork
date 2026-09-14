#!/usr/bin/env bash
# Caller must explicitly supply SIM_RUN_AUTHORIZED=1. One run only, no retries.
set -e
task_dir=$(cd "${BASH_SOURCE[0]%/*}" && pwd)
task_root=$(cd "$task_dir/../../.." && pwd)
seed="${1:?seed32 or seed34 required}"
mode="${2:?strategy or baseline required}"
case "$seed" in 32|34) ;; *) exit 64;; esac
case "$mode" in strategy) strategy=true;; baseline) strategy=false;; *) exit 64;; esac
cd "$task_root"
export ASTRA_MODEL_ROOT="$task_root/simulation_assets/models"
export GAZEBO_MODEL_PATH="$task_root/vision_ws/src/uav_vision_eval/models:$ASTRA_MODEL_ROOT"
export SIM_STORAGE_GUARD_PATH="${SIM_STORAGE_GUARD_PATH:-/mnt/f}"
export SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1
exec timeout --signal=TERM --kill-after=30s 2850s bash top_level_scripts/sim_run.sh \
  "fast_random_${mode}_seed${seed}" bash top_level_scripts/roslaunch_rl_drone.sh \
  uav_high_view fast_comparison.launch "strategy:=$strategy" "field_seed:=$seed" \
  "target_model_path:=${UAV_VISION_MODEL_PATH:-/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt}" \
  "world:=$task_dir/seed_$seed/field.world" \
  "field_config:=$task_dir/seed_$seed/field_config.yaml" \
  "runtime_config:=$task_dir/seed_$seed/runtime.yaml" \
  "gate_geometry_config:=$task_dir/seed_$seed/gate_geometry.yaml"
