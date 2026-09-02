#!/usr/bin/env bash
# Run a named V-SIM-04 operating-surface slice through the unified run logger.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SLICE="${1:?usage: run_vsim04_surface.sh static25|sparse30|b100|c25 [name:=value ...]}"
shift

case "${SLICE}" in
  static25|sparse30|b100|c25) ;;
  *)
    echo "unknown V-SIM-04 surface slice: ${SLICE}" >&2
    exit 2
    ;;
esac

# Remaining arguments are an explicit roslaunch passthrough. Protect the
# wrapper-owned matrix/slice selection; roslaunch itself rejects unknown names.
for LAUNCH_ARGUMENT in "$@"; do
  case "${LAUNCH_ARGUMENT}" in
    matrix_file:=*|trial_slice:=*|trial_selector:=*)
      echo "surface wrapper owns matrix_file/trial_slice/trial_selector" >&2
      exit 2
      ;;
    *:=*) ;;
    *)
      echo "expected roslaunch name:=value argument: ${LAUNCH_ARGUMENT}" >&2
      exit 2
      ;;
  esac
done

MODEL_PATH="${UAV_VISION_MODEL_PATH:?set UAV_VISION_MODEL_PATH to the dev/sim .pt model}"
if [ ! -f "${MODEL_PATH}" ]; then
  echo "UAV_VISION_MODEL_PATH is not a file: ${MODEL_PATH}" >&2
  exit 2
fi

export VSIM04_VISION_REVISION="${VSIM04_VISION_REVISION:-$(git -C "${PROJECT_ROOT}" rev-parse HEAD)}"
export VSIM04_NAVIGATION_REVISION="${VSIM04_NAVIGATION_REVISION:?set VSIM04_NAVIGATION_REVISION}"
export SIM_NO_RECORD="${SIM_NO_RECORD:-1}"

if [ "${SLICE}" = "c25" ]; then
  MATRIX_FILE="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/vsim04_lateral_c25_matrix.yaml"
  SCENE="vsim04_c25_seed11"
  bash "${SCRIPT_DIR}/sim_run.sh" "${SCENE}" \
    roslaunch uav_vision_eval vsim04_stability.launch \
    gui:=false matrix_file:="${MATRIX_FILE}" "$@"
else
  MATRIX_FILE="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/vsim04_operating_surface_matrix.yaml"
  SCENE="vsim04_diag_${SLICE}_seed11"
  bash "${SCRIPT_DIR}/sim_run.sh" "${SCENE}" \
    roslaunch uav_vision_eval vsim04_stability.launch \
    gui:=false matrix_file:="${MATRIX_FILE}" trial_slice:="${SLICE}" "$@"
fi
