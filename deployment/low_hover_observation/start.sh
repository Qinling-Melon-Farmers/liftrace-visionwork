#!/usr/bin/env bash
set -euo pipefail
here="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
root="$(cd "$here/../.." && pwd)"
mode="${1:-preview}"
profile="${2:-hover}"
[[ $# -gt 0 ]] && shift
[[ $# -gt 0 ]] && shift
case "$mode" in
  preview) exec "${BOARD_PYTHON:-python3}" "$here/flight.py" preview --profile "$profile" "$@" ;;
  localization|flight|record) ;;
  *) echo 'Usage: start.sh preview|localization|flight|record [hover|forward|square] [options]' >&2; exit 2 ;;
esac
set +u
source "$root/deployment/site_20260928/environment.sh"
set -u
expected="$root/patrol_uav_ws-patrol_planner/src/uav_mission"
actual="$(rospack find uav_mission)"
[[ "$(readlink -f "$actual")" == "$(readlink -f "$expected")" ]] || { echo 'Wrong uav_mission overlay'; exit 2; }
case "$mode" in
  localization)
    "${BOARD_PYTHON:-/usr/bin/python3}" "$here/flight.py" localization-check --profile "$profile"
    exec roslaunch "$here/localization.launch" "$@" ;;
  flight)
    exec "${BOARD_PYTHON:-/usr/bin/python3}" "$here/flight.py" flight --profile "$profile" "$@" ;;
  record)
    exec "${BOARD_PYTHON:-/usr/bin/python3}" "$here/record_diagnostics.py" --config "$here/recording.yaml" "$@" ;;
esac
