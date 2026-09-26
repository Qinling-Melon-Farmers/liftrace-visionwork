#!/usr/bin/env bash
set -euo pipefail
tool_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd)
if (( $# < 2 )); then
  echo 'Usage: bash run.sh INPUT.bag OUTPUT_DIR [--fps 10] [--frame map] [--topics topics.json]'
  exit 2
fi
bag=$1
output=$2
shift 2
set +u
source "${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
"${ROS_PYTHON:-/usr/bin/python3}" "$tool_dir/bag_replay.py" export --bag "$bag" --out "$output" "$@"
if [[ -z "${REPLAY_PYTHON:-}" ]]; then
  if [[ -f /home/xhj/miniconda3/etc/profile.d/conda.sh ]]; then
    source /home/xhj/miniconda3/etc/profile.d/conda.sh
    conda activate rl_drone
    REPLAY_PYTHON=$(command -v python)
  else
    REPLAY_PYTHON=$(command -v python3)
  fi
fi
set -u
"$REPLAY_PYTHON" "$tool_dir/bag_replay.py" render --out "$output" "$@"
"$REPLAY_PYTHON" "$tool_dir/bag_replay.py" verify --out "$output" "$@"
