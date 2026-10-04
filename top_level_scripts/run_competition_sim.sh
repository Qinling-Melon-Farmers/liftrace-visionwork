#!/usr/bin/env bash
# Research default: clear inner 10m, original fixed four trees, seed-specific targets.
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
if [[ "${1:-}" == --legacy ]]; then
  shift
  exec "$script_dir/run_competition_sim_legacy.sh" "$@"
fi
exec "$script_dir/run_high_view_sim.sh" "$@"
