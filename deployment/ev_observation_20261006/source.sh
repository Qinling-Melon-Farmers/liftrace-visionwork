#!/usr/bin/env bash
# Source only: identical workspace dependency selection to the low-hover suite.
EV_OBSERVATION_DIR="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
EV_OBSERVATION_ROOT="$(cd "$EV_OBSERVATION_DIR/../.." && pwd)"
source "$EV_OBSERVATION_ROOT/deployment/site_20260928/environment.sh"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
# Only this Python predictor process; does not change the running LIO process.
export PYTHONPATH="$EV_OBSERVATION_DIR:$EV_OBSERVATION_DIR/source/patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts:${PYTHONPATH:-}"
# Never add vendored uav_mission ahead of the current generated ROS package.
