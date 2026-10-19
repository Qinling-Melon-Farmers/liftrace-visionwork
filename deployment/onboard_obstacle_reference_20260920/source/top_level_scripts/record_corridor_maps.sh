#!/usr/bin/env bash
# Read-only obstacle-map ROS recorder. Full-rate PointCloud2 + navigation evidence. Does not start drivers, mission nodes or control services.
set -euo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# 默认不限时长：一直记录直到 Ctrl+C；显式传参才启用定时（1-3600 秒）。
DURATION="${1:-}"
if [[ "$DURATION" == -h || "$DURATION" == --help ]]; then
  echo "Usage: bash $0 [duration_seconds=until Ctrl+C] [output_parent=PROJECT/logs]"
  exit 0
fi
if [[ -n "$DURATION" ]] && { [[ ! "$DURATION" =~ ^[1-9][0-9]*$ ]] || (( DURATION > 3600 )); }; then
  echo 'Duration must be an integer from 1 to 3600 seconds.' >&2
  exit 2
fi
# Source the existing overlay for custom ROS message definitions.
set +u
source "$PROJECT_ROOT/patrol_uav_ws-patrol_planner/devel/setup.bash"
set -u
for cmd in rosbag rosparam rosnode rostopic timeout grep; do
  command -v "$cmd" >/dev/null || { echo "Missing command: $cmd" >&2; exit 1; }
done
if ! timeout 5 rosnode list >/dev/null 2>&1; then
  echo 'ROS master unavailable; start your existing ROS system before recording.' >&2
  exit 1
fi
OUTPUT_PARENT="${2:-$PROJECT_ROOT/logs}"
mkdir -p "$OUTPUT_PARENT"
RUN_DIR="$(mktemp -d "$OUTPUT_PARENT/corridor_map_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
RUN_DIR="$(cd "$RUN_DIR" && pwd)"
# Check the common typo as well; keep the canonical subscription for late publishers.
timeout 5 rostopic list > "$RUN_DIR/topics_at_start.txt" 2>&1 || true
CLOUD_TOPIC=/freedom/static_pointcloud
if ! grep -Fxq /freedom/static_pointcloud "$RUN_DIR/topics_at_start.txt" && \
   grep -Fxq /freedom/static_pointcloude "$RUN_DIR/topics_at_start.txt"; then
  CLOUD_TOPIC=/freedom/static_pointcloude
fi
TOPICS=(
  "$CLOUD_TOPIC" /sdf_map/occupancy /sdf_map/occupancy_inflate
  /Odometry /mavros/vision_pose/pose /mavros/local_position/pose
  /mavros/local_position/odom /navigation/local_pose /navigation/local_odom
  /navigation/setpoint_mission
  /mavros/state /mavros/extended_state /mavros/statustext/recv
  /mavros/setpoint_position/local /mavros/setpoint_raw/target_local
  /fastplanner/goal /fastplanner/setpoint_position/local
  /planning/goal_status /planning/bspline /planning/pos_cmd /planning/replan /planning/new
  /navigation/mission_command_raw /navigation/mission_result
  /navigation/mission_status /navigation/planner_bridge_status
  /mission/command /mission/control_ready /detect/point_class /uav_vision/align_mode
  /tf /tf_static /clock /rosout_agg
)
printf '%s\n' "${TOPICS[@]}" > "$RUN_DIR/topics.txt"
{
  date --iso-8601=seconds
  echo "duration_seconds=${DURATION:-unlimited}"
  echo 'Read-only capture; bag receipt time and message header stamps are both retained.'
  echo "cloud_topic=$CLOUD_TOPIC"
  echo 'Full-rate PointCloud2; no resampling or coordinate conversion.'
  echo 'Occupancy topics are published visualization clouds, not a complete serialized SDF grid.'
  echo 'Visualization height/range filters apply; /sdf_map/unknown is not recorded as ground-truth free space.'
  df -h "$RUN_DIR"
} > "$RUN_DIR/metadata.txt"

snapshot() {
  local phase="$1" key
  mkdir -p "$RUN_DIR/$phase"
  date --iso-8601=seconds > "$RUN_DIR/$phase/time.txt"
  timeout 5 rosnode list > "$RUN_DIR/$phase/nodes.txt" 2>&1 || true
  timeout 5 rostopic list -v > "$RUN_DIR/$phase/topics_graph.txt" 2>&1 || true
  # Explicit allowlist: never dump the entire parameter server or environment.
  for key in /use_sim_time /waypoints /navigation/mission_manager/mission \
    /navigation/mission_manager/search/altitude /navigation/mission_manager/runtime \
    /navigation/planner_bridge/execution /fast_planner_node/fsm \
    /fast_planner_node/sdf_map /fast_planner_node/manager /freedom/sensor /freedom/map /external_mission_mode /external_planner_max_command_z \
    /px4_max_distance /threshould/aligning_threshould /switch/flag_planner_px4; do
    timeout 2 rosparam get "$key" > "$RUN_DIR/$phase/${key//\//_}.yaml" 2>&1 || true
  done
}
RECORDER_PID=''
TIMER_PID=''
finish() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$TIMER_PID" ]]; then
    kill "$TIMER_PID" 2>/dev/null || true
    wait "$TIMER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RECORDER_PID" ]]; then
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
  fi
  snapshot end
  date --iso-8601=seconds >> "$RUN_DIR/metadata.txt"
  echo "exit_code=$result" >> "$RUN_DIR/metadata.txt"
  shopt -s nullglob
  local bags=("$RUN_DIR"/*.bag) active=("$RUN_DIR"/*.bag.active)
  for bag in "${bags[@]}"; do
    rosbag info "$bag" >> "$RUN_DIR/bag_info.txt" 2>&1 || true
  done
  if (( ${#active[@]} > 0 || ${#bags[@]} == 0 )); then
    echo 'Capture incomplete: inspect recorder.log and any .bag.active files.' >&2
    result=1
  fi
  if (( ${#bags[@]} > 0 )); then
    /usr/bin/python3 - "$RUN_DIR" "$CLOUD_TOPIC" <<'CHECK_BAGS'
import glob, json, os, sys
import rosbag
root, cloud = sys.argv[1:]
counts = {name: 0 for name in (cloud, '/sdf_map/occupancy', '/sdf_map/occupancy_inflate')}
for filename in glob.glob(os.path.join(root, '*.bag')):
    with rosbag.Bag(filename) as bag:
        for name, info in bag.get_type_and_topic_info().topics.items():
            if name in counts:
                counts[name] += info.message_count
with open(os.path.join(root, 'map_topic_counts.json'), 'w') as output:
    json.dump(counts, output, indent=2)
for name, count in counts.items():
    print('%s: %d messages%s' % (name, count, ' (MISSING: map evidence incomplete)' if not count else ''))
CHECK_BAGS
  fi
  echo "Diagnostics saved: $RUN_DIR"
  exit "$result"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Record before snapshots so one-shot goal/lifecycle events are not missed during inspection.
rosbag record --buffsize=128 --split --size=256 --repeat-latched -O "$RUN_DIR/obstacle_map.bag" \
  "${TOPICS[@]}" > "$RUN_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
if [[ -n "$DURATION" ]]; then
  (
    sleep "$DURATION" &
    SLEEP_PID=$!
    trap 'kill "$SLEEP_PID" 2>/dev/null || true; wait "$SLEEP_PID" 2>/dev/null || true; exit 0' TERM INT
    wait "$SLEEP_PID"
    kill -INT "$RECORDER_PID" 2>/dev/null || true
  ) &
  TIMER_PID=$!
  echo "Recording for $DURATION seconds: $RUN_DIR"
else
  echo "Recording until Ctrl+C: $RUN_DIR"
fi
echo "Map topics: $CLOUD_TOPIC /sdf_map/occupancy /sdf_map/occupancy_inflate"
echo 'Full-rate maps can produce large bags; files split at 256 MB without deleting older parts.'
echo 'Start capture before starting the mission. Ctrl-C stops only this recorder.' 
snapshot start
wait "$RECORDER_PID"
RECORDER_PID=''
