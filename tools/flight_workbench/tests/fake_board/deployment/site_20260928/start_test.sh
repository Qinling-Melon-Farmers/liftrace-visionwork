#!/usr/bin/env bash
# 纸板现场快捷入口（自检用）：只把 1..6 映射到专项名，然后打印同一份纸板输出。
set -euo pipefail
site_dir="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
trial="${1:-}"
mode="${2:-preview}"
case "$trial" in
  1|single) folder=visual_interrupt ;;
  2|multi) folder=low_multi ;;
  3|memory) folder=memory_only ;;
  4|revisit) folder=high_view_revisit ;;
  5|priority) folder=high_priority ;;
  6|capture) folder=high_speed_capture ;;
  *) echo "Usage: start_test.sh 1|2|3|4|5|6 [preview|flight]" >&2; exit 2 ;;
esac
exec bash "$site_dir/../../emit_transcript.sh" "$folder" "$mode"
