#!/usr/bin/env bash
# 建立完整 ROS/PX4 overlay 后运行无 GUI 的单下视—斜下辅助重复 A/B。
set -euo pipefail

SCRIPT_DIR="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
PACKAGE_DIR="${SCRIPT_DIR%/*}"
PROJECT_ROOT="${PACKAGE_DIR%/vision_ws/src/uav_vision_eval}"

set +u
source "${PROJECT_ROOT}/top_level_scripts/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
set -u

export SIM_NO_RECORD=1
python3 "${SCRIPT_DIR}/run_oblique_search_ab.py" "$@"
