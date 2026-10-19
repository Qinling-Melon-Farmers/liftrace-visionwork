#!/usr/bin/env bash
# 独立临时试验；preview/配置检查分支绝不加载或启动 ROS。
set -euo pipefail
here="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
root="$(cd "$here/../../.." && pwd)"
mode="${1:-preview}"
[[ $# -gt 0 ]] && shift
check=false
for arg in "$@"; do
  [[ "$arg" == '--check-config' ]] && check=true
done
if [[ "$mode" == preview || "$check" == true ]]; then
  exec "${BOARD_PYTHON:-python3}" "$here/entry.py" "$mode" "$@"
fi
[[ "$mode" == flight ]] || { echo 'Usage: start.sh preview|flight --slot 1 [--real-release] [--check-config] [--config PATH] [--output PATH]' >&2; exit 2; }
# 先离线校验所有参数；无效参数不能触发任何节点启动。
"${BOARD_PYTHON:-python3}" "$here/entry.py" "$mode" "$@" --check-config
set +u
source "$root/deployment/site_20260928/environment.sh"
set -u
for package in uav_mission actuator_pwm patrol_control; do
  actual="$(rospack find "$package")"
  expected="$root/patrol_uav_ws-patrol_planner/src/$package"
  [[ "$(readlink -f "$actual")" == "$(readlink -f "$expected")" ]] || { echo "Wrong $package overlay" >&2; exit 2; }
done
exec "${BOARD_PYTHON:-/usr/bin/python3}" "$here/entry.py" "$mode" "$@"
