#!/usr/bin/env bash
# Run baseline/a68925d preflight and fixed-route samples through sim_run.sh.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_RUN="${SCRIPT_DIR}/sim_run.sh"
AB_COMPARE="${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_ab_compare.py"
FIELD_SEED="${NAV_AB_FIELD_SEED:-11}"
CLASS_PROFILE="${NAV_AB_CLASS_PROFILE:-r2026}"
WORLD="${NAV_AB_WORLD:-${PROJECT_ROOT}/toudi3_random.world}"
TARGET_MODEL_PATH="${UAV_VISION_MODEL_PATH:-}"

if [ -z "${TARGET_MODEL_PATH}" ] || [ ! -f "${TARGET_MODEL_PATH}" ]; then
  echo "UAV_VISION_MODEL_PATH must name an existing model file" >&2
  exit 2
fi
if [ ! -f "${WORLD}" ]; then
  echo "navigation A/B world is missing: ${WORLD}" >&2
  exit 2
fi

latest_run_dir() {
  local scene="$1"
  local matches=()
  shopt -s nullglob
  matches=("${PROJECT_ROOT}/logs/${scene}_"*)
  shopt -u nullglob
  if [ "${#matches[@]}" -eq 0 ]; then
    return 1
  fi
  printf '%s\n' "${matches[${#matches[@]}-1]}"
}

run_preflight() {
  local profile="$1"
  local scene="vcl06_preflight_seed${FIELD_SEED}_${profile}"
  SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 bash "${SIM_RUN}" "${scene}" \
    roslaunch uav_mission navigation_random_field_preflight.launch \
    world:="${WORLD}" target_model_path:="${TARGET_MODEL_PATH}" \
    field_seed:="${FIELD_SEED}" class_profile:="${CLASS_PROFILE}" \
    nav_feature_profile:="${profile}" gui:=false rviz:=false >&2
  latest_run_dir "${scene}"
}

run_sample() {
  local profile="$1"
  local scene="vcl06_ab90_seed${FIELD_SEED}_${profile}"
  SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 bash "${SIM_RUN}" "${scene}" \
    roslaunch uav_mission navigation_random_field_ab.launch \
    world:="${WORLD}" target_model_path:="${TARGET_MODEL_PATH}" \
    field_seed:="${FIELD_SEED}" class_profile:="${CLASS_PROFILE}" \
    nav_feature_profile:="${profile}" gui:=false rviz:=false >&2
  latest_run_dir "${scene}"
}

run_preflight baseline
BASELINE_RUN="$(run_sample baseline)"
run_preflight a68925d
CANDIDATE_RUN="$(run_sample a68925d)"

BASELINE_METRICS="${BASELINE_RUN}/ab_metrics.json"
CANDIDATE_METRICS="${CANDIDATE_RUN}/ab_metrics.json"
COMPARE_SCENE="vcl06_ab_compare_seed${FIELD_SEED}"
SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 bash "${SIM_RUN}" "${COMPARE_SCENE}" \
  python3 "${AB_COMPARE}" \
  "${BASELINE_METRICS}" "${CANDIDATE_METRICS}"

echo "baseline metrics: ${BASELINE_METRICS}"
echo "candidate metrics: ${CANDIDATE_METRICS}"
echo "candidate promotion remains a separate reviewed commit"
