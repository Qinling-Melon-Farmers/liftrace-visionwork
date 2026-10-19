#!/usr/bin/env bash
set -euo pipefail
site_dir="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
trial="${1:-}"
mode="${2:-preview}"
case "$trial" in
  1|single) folder=01_visual_interrupt ;;
  2|multi) folder=05_low_multi ;;
  3|memory) folder=07_memory_only ;;
  4|revisit) folder=02_high_view_revisit ;;
  5|priority) folder=06_high_priority ;;
  6|capture) folder=09_high_speed_capture ;;
  h|landing) folder=03_h_landing ;;
  *) echo "Usage: start_test.sh 1|2|3|4|5|6|h|landing [preview|flight]"; exit 2 ;;
esac
if [[ "$folder" == 03_h_landing ]]; then
  shift "$(( $# >= 2 ? 2 : $# ))"
  # H has no delivery entry. Keep its dedicated profile even with extra options.
  exec bash "$site_dir/../board_trials_4x4/$folder/start.sh" "$mode" "$@" --site-config "$site_dir/h_landing_test_area.yaml"
fi
if [[ "$folder" == 09_high_speed_capture ]]; then
  shift "$(( $# >= 2 ? 2 : $# ))"
  exec bash "$site_dir/../board_trials_4x4/$folder/start.sh" "$mode" --site-config "$site_dir/test_area.yaml" "$@"
fi
[[ $# -le 2 ]] || { echo "Use the explicit per-module start_real.sh for a separately prepared real release"; exit 2; }
if [[ "$mode" == flight && "$folder" != 07_memory_only ]]; then
  echo "SITE 2026-09-28: REAL release selected by operator; guarded Servo chain."
  exec bash "$site_dir/../board_trials_4x4/$folder/start_real.sh" --site-config "$site_dir/test_area.yaml"
fi
exec bash "$site_dir/../board_trials_4x4/$folder/start.sh" "$mode" --site-config "$site_dir/test_area.yaml"
