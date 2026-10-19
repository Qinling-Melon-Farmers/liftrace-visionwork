#!/usr/bin/env bash
# Explicit commands only; never deploy, install packages, or select weights automatically.
set -euo pipefail
SCRIPT_DIR="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
CONDA_INIT="${MODEL_RELEASE_CONDA_INIT:-/home/xhj/miniconda3/etc/profile.d/conda.sh}"
[[ -f "$CONDA_INIT" ]] || { echo "Missing existing conda activation script: $CONDA_INIT" >&2; exit 2; }
source "$CONDA_INIT"
conda activate rl_drone
export YOLO_AUTOINSTALL=false
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
exec python "$SCRIPT_DIR/release.py" "$@"
