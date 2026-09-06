#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
: "${UAV_VISION_MODEL_PATH:?Set UAV_VISION_MODEL_PATH to the six-class YOLO weights}"
export UAV_WS="$PROJECT_ROOT/patrol_uav_ws-patrol_planner"
export VISION_WS="$PROJECT_ROOT/vision_ws"
export SIM_REQUIRE_GATE=1
exec "$SCRIPT_DIR/sim_run.sh" "${SIM_SCENE:-r2026_full}" \
  roslaunch uav_mission navigation_horizontal_search_vcl06.launch \
  "target_model_path:=${UAV_VISION_MODEL_PATH}" "$@"
