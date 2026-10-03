#!/usr/bin/env bash
# 纸板专项入口：按 run_trial.py 的真实输出格式打印一遍再退出（只用于本机自检）。
set -u
trial="${1:-high_priority}"
mode="${2:-flight}"
log_dir="$PWD/logs/board_${trial}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$log_dir"
echo "INITIALIZING {'pose_samples': 6, 'camera_info': True, 'image_seen': True, 'compressed_fresh': True, 'mapping_alignment': {'reason': 'settling', 'stable_for': 0.3}, 'armed': False, 'yaw_deg': 0.9, 'xyz_span': [0.01, 0.008, 0.004]}"
sleep 0.3
echo "INITIALIZING {'pose_samples': 74, 'camera_info': True, 'image_seen': True, 'compressed_fresh': True, 'mapping_alignment': {'reason': 'stable'}, 'armed': False, 'yaw_deg': 0.94, 'xyz_span': [0.006, 0.005, 0.003]}"
sleep 0.3
echo "MAPPING_READY {'mapping_started': 10.0, 'map_ready': 12.4, 'first_cloud': 10.4, 'last_cloud': 12.3, 'distinct_clouds': 18, 'alignment': {'reason': 'stable'}}"
sleep 0.3
echo "READY: $log_dir"
echo "Ground/reference and all local-Z limits generated automatically. No arming or mission start was sent."
if [ "$mode" = "flight" ]; then
  echo "AUTO_SEQUENCE: manual arm -> OFFBOARD -> low hover -> mission. Never auto-arms."
  sleep 0.3
  echo "FLIGHT_STATUS {'mode': 'AUTO.LOITER', 'armed': False, 'phase': 'IDLE', 'reason': ''}"
  sleep 0.3
  echo "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': True, 'phase': 'LOW_HOVER', 'reason': ''}"
  echo "AUTO_MISSION_START True "
  sleep 0.3
  echo "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': True, 'phase': 'HIGH_VIEW_SEARCH', 'reason': 'ok'}"
  sleep 0.3
fi
printf '{"end_reason": "landed_after_flight", "ever_armed": true, "trial": "%s"}\n' "$trial" >"$log_dir/supervisor_result.json"
printf '{"result": "selftest", "trial": "%s"}\n' "$trial" >"$log_dir/result.json"
echo "Trial application stopped; device MAVROS/driver2 left running. Logs: $log_dir"
