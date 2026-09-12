#!/usr/bin/env bash
# Explicit launch only: does not arm or start the navigation mission.
set -e
script_dir="${BASH_SOURCE[0]%/*}"
project_root="$(cd "$script_dir/.." && pwd)"
mode="${1:-}"
ground_z="${2:-}"
if [[ "$mode" != preview && "$mode" != flight ]] || [[ -z "$ground_z" ]]; then
  echo "Usage: $0 preview|flight MEASURED_GROUND_Z" >&2
  echo "GROUND_Z is the measured ground in camera_init, not an assumed SITL offset." >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$project_root/vision_ws/devel/setup.bash"
source "$project_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
/usr/bin/python3 - "$ground_z" <<'PY'
import math, sys
z = float(sys.argv[1])
if not math.isfinite(z) or z + .5 <= .05:
    raise SystemExit('Invalid ground_z: cruise local Z must exceed 0.05 for legacy control.')
print('FC cruise: 0.50 m AGL; local Z = %.3f m' % (z + .5))
PY
nodes="$(rosnode list 2>/dev/null)" || {
  echo "Start the device-side MAVROS and driver2 first." >&2; exit 2;
}
for node in /laserMapping /freedom /patrol_control /fast_planner_node /navigation_frame_adapter; do
  if [[ $'\n'"$nodes"$'\n' == *$'\n'"$node"$'\n'* ]]; then
    echo "Existing application node $node; stop the old application before using this entry." >&2
    exit 2
  fi
done
output=false
[[ "$mode" == flight ]] && output=true
mkdir -p "$project_root/logs"
run_dir="$(mktemp -d "$project_root/logs/obstacle4x4_$(date +%Y%m%d_%H%M%S)_XXXX")"
printf 'mode: %s\nground_z: %s\ncruise_agl: 0.50\n' "$mode" "$ground_z" > "$run_dir/config.yaml"
export ROS_LOG_DIR="$run_dir/roslog"
echo "Logs: $run_dir"
exec roslaunch uav_mission obstacle_4x4_hardware.launch \
  "ground_z:=$ground_z" "enable_control_output:=$output"
