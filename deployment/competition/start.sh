#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
source "$SCRIPT_DIR/environment.sh"
exec /usr/bin/python3 "$COMPETITION_ROOT/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py" "$@" --root "$COMPETITION_ROOT"
