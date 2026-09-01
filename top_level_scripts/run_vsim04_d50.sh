#!/usr/bin/env bash
# Run one D50 supported single-target trial or the supported diagnostic slice.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SELECTION="${1:-d_single_01}"
if [ "$#" -gt 0 ]; then
  shift
fi

case "${SELECTION}" in
  supported)
    SCENE_TOKEN="supported"
    ;;
  d_single_[0-9][0-9])
    TRIAL_NUMBER="${SELECTION#d_single_}"
    if [ "${TRIAL_NUMBER}" -lt 1 ] || [ "${TRIAL_NUMBER}" -gt 40 ]; then
      echo "D50 single trial must be in d_single_01..d_single_40" >&2
      exit 2
    fi
    SCENE_TOKEN="${SELECTION}"
    ;;
  *)
    echo "usage: run_vsim04_d50.sh d_single_01..d_single_40|supported [name:=value ...]" >&2
    exit 2
    ;;
esac

for LAUNCH_ARGUMENT in "$@"; do
  case "${LAUNCH_ARGUMENT}" in
    matrix_file:=*|trial_slice:=*|trial_selector:=*|trial_runner_type:=*)
      echo "D50 wrapper owns matrix/selection/runner arguments" >&2
      exit 2
      ;;
    *:=*) ;;
    *)
      echo "expected roslaunch name:=value argument: ${LAUNCH_ARGUMENT}" >&2
      exit 2
      ;;
  esac
done

MATRIX_FILE="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/vsim04_trajectory_d50_matrix.yaml"
VISION_PYTHON_BIN="${VISION_PYTHON:-/home/xhj/miniconda3/envs/rl_drone/bin/python}"
if [ ! -x "${VISION_PYTHON_BIN}" ]; then
  echo "VISION_PYTHON is not executable: ${VISION_PYTHON_BIN}" >&2
  exit 2
fi
D50_PYTHONPATH="${PROJECT_ROOT}/vision_ws/src/uav_vision/src:${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/src"
SUPPORTED_TRIALS="$(PYTHONPATH="${D50_PYTHONPATH}" "${VISION_PYTHON_BIN}" \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/scripts/vsim04_d50_dry_run.py" \
  --matrix "${MATRIX_FILE}" --list-runtime-supported)"

if [ "${SELECTION}" = "supported" ]; then
  mapfile -t SUPPORTED_TRIAL_IDS <<< "${SUPPORTED_TRIALS}"
  SUPPORTED_SELECTOR="$(IFS=,; echo "${SUPPORTED_TRIAL_IDS[*]}")"
  if [ -z "${SUPPORTED_SELECTOR}" ]; then
    echo "D50 runtime-supported trial list is empty" >&2
    exit 2
  fi
  SELECTION_ARGUMENTS=("trial_selector:=${SUPPORTED_SELECTOR}")
else
  SELECTION_SUPPORTED=0
  while IFS= read -r SUPPORTED_TRIAL; do
    if [ "${SUPPORTED_TRIAL}" = "${SELECTION}" ]; then
      SELECTION_SUPPORTED=1
    fi
  done <<< "${SUPPORTED_TRIALS}"
  if [ "${SELECTION_SUPPORTED}" != "1" ]; then
    echo "D50 trial is NOT_RUN by the current framing/arena contract: ${SELECTION}" >&2
    exit 2
  fi
  SELECTION_ARGUMENTS=("trial_selector:=${SELECTION}")
fi

MODEL_PATH="${UAV_VISION_MODEL_PATH:?set UAV_VISION_MODEL_PATH to the dev/sim .pt model}"
if [ ! -f "${MODEL_PATH}" ]; then
  echo "UAV_VISION_MODEL_PATH is not a file: ${MODEL_PATH}" >&2
  exit 2
fi

export VSIM04_VISION_REVISION="${VSIM04_VISION_REVISION:-$(git -C "${PROJECT_ROOT}" rev-parse HEAD)}"
export VSIM04_NAVIGATION_REVISION="${VSIM04_NAVIGATION_REVISION:?set VSIM04_NAVIGATION_REVISION}"
export SIM_NO_RECORD="${SIM_NO_RECORD:-1}"

bash "${SCRIPT_DIR}/sim_run.sh" "vsim04_diag_d50_${SCENE_TOKEN}_seed11" \
  roslaunch uav_vision_eval vsim04_d50_single.launch \
  gui:=false "${SELECTION_ARGUMENTS[@]}" "$@"
