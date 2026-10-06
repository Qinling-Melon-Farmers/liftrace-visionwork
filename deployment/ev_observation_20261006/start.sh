#!/usr/bin/env bash
set -euo pipefail
ev_here="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
ev_mode="${1:-preview}"
if [[ $# -gt 0 ]]; then shift; fi
export PYTHONDONTWRITEBYTECODE=1
case "$ev_mode" in
  preview)
    exec "${EV_OFFLINE_PYTHON:-python3}" "$ev_here/observer.py" preview "$@" ;;
  offline)
    exec "${EV_OFFLINE_PYTHON:-python3}" "$ev_here/offline.py" "$@" ;;
  record-preview)
    exec "${EV_OFFLINE_PYTHON:-python3}" "$ev_here/record_shadow.py" "$@" ;;
  check|ros-test|shadow|reset|graph-check|record)
    set +u
    source "$ev_here/source.sh"
    set -u
    case "$ev_mode" in
      ros-test) exec /usr/bin/python3 "$ev_here/offline.py" --ros "$@" ;;
      record) exec /usr/bin/python3 "$ev_here/record_shadow.py" "$@" ;;
      *) exec /usr/bin/python3 "$ev_here/observer.py" "$ev_mode" "$@" ;;
    esac ;;
  parallel-check)
    exec "${EV_OFFLINE_PYTHON:-python3}" "$ev_here/parallel_check.py" "$@" ;;
  *) echo 'Usage: start.sh preview|offline|check|ros-test|shadow|reset|graph-check|parallel-check|record-preview|record' >&2; exit 2 ;;
esac
