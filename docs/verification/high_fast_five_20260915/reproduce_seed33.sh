#!/bin/bash
# seed33 high fast-search single-round reproduction (same scenario/entry as the 2026-09-15 batch; code = current HEAD 59c6215)
# Mirrors one matrix.py invocation: SIM_REQUIRE_GATE, no recording, 2850s timeout, unified cleanup.
# Run only after explicit current-request authorization; this script does not auto-retry.
set -u
R=/home/xhj/liftrace-worktrees/r2026-high-view-search
cd "$R" || exit 1
CASE="$R/docs/verification/high_fast_five_20260915/seed_33"
MODEL=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt

# pre-flight zero-residual check (rule 19: the only accepted check method)
bash top_level_scripts/check_sim_processes.sh || exit 1

env SIM_RUN_AUTHORIZED=1 SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 SIM_STORAGE_GUARD_PATH=/mnt/f \
  ASTRA_MODEL_ROOT="$R/simulation_assets/models" \
  GAZEBO_MODEL_PATH="$R/vision_ws/src/uav_vision_eval/models:$R/simulation_assets/models" \
  timeout --signal=TERM --kill-after=30s 2850s \
  bash top_level_scripts/sim_run.sh seed33_repro_20260915 \
    bash top_level_scripts/roslaunch_rl_drone.sh uav_high_view fast_comparison.launch \
    strategy:=true field_seed:=33 \
    target_model_path:="$MODEL" \
    world:="$CASE/field.world" \
    field_config:="$CASE/field_config.yaml" \
    runtime_config:="$CASE/runtime.yaml" \
    gate_geometry_config:="$CASE/gate_geometry.yaml"
rc=$?
echo "SIM_RUN_EXIT=$rc"
exit $rc