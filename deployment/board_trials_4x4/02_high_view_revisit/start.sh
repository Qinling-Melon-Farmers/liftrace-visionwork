#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
exec bash "$script_dir/../common/uav_board_trials/scripts/start_trial.sh" high_view "$@"
