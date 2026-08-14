#!/usr/bin/env bash
# 生成派生机架后，经统一 sim_run.sh 运行覆盖航线 shadow。
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PACKAGE_DIR="${SCRIPT_DIR%/*}"
PROJECT_ROOT="${PACKAGE_DIR%/vision_ws/src/uav_vision_eval}"

angle="${1:-55}"
mode="${2:-mono}"
gui="${3:-false}"
if [[ "${angle}" != "45" && "${angle}" != "55" && "${angle}" != "60" ]]; then
  echo "angle must be 45, 55 or 60" >&2
  exit 2
fi
if [[ "${mode}" != "mono" && "${mode}" != "depth" ]]; then
  echo "mode must be mono or depth" >&2
  exit 2
fi

set +u
source "${PROJECT_ROOT}/top_level_scripts/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
set -u
base_sdf="${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/src/patrol_control/models/iris_mid360_downward_camera/model.sdf"
derived_sdf="/tmp/iris_mid360_downward_aux_camera_${angle}_$$.sdf"
python3 "${SCRIPT_DIR}/generate_oblique_vehicle_sdf.py" \
  --base-sdf "${base_sdf}" --output "${derived_sdf}" --angle-deg "${angle}"

cleanup() {
  rm -f "${derived_sdf}"
}
trap cleanup EXIT

scene="oblique_shadow_${mode}_${angle}deg"
bash "${PROJECT_ROOT}/top_level_scripts/sim_run.sh" "${scene}" \
  roslaunch uav_vision_eval oblique_coverage_shadow.launch \
  gui:="${gui}" start_arming:=true projection_mode:="${mode}" \
  aux_angle_deg:="${angle}" vehicle_sdf:="${derived_sdf}"
