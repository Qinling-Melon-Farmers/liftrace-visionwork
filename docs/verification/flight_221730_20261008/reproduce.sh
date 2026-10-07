#!/usr/bin/env bash
# Offline only; no node launch, ROS publication, board connection or actuator action.
set -euo pipefail
report_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd)
root=$(cd -- "$report_dir/../../.." && pwd)
cd "$root"
run=$(find "$root" -maxdepth 2 -type d -name board_full_mission_20261007_221730)
[[ -n "$run" && -f "$run/flight_debug_0.bag" ]]
mkdir -p "$run/replay" "$run/analysis"
bash tools/bag_replay/run.sh "$run/flight_debug_0.bag" "$run/replay" --fps 5 --frame camera_init --keep-frames > "$run/analysis/replay_command.log" 2>&1
set +u
source /opt/ros/noetic/setup.bash
/usr/bin/python3 "$report_dir/extract_evidence.py" --run "$run"
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
set -u
python - "$run" <<'PY'
import json,sys
from pathlib import Path
r=Path(sys.argv[1]);d=json.loads((r/'replay/data.json').read_text())
frames=[dict(f,file=str((r/'replay'/f['file']).resolve())) for f in d['frames'] if f['stamp']>=275]
(r/'analysis/h_frames.json').write_text(json.dumps(frames),encoding='utf-8')
PY
python tools/bag_replay/h_geometry_offline.py --frames "$run/analysis/h_frames.json" --output "$run/analysis/h_otsu" --segmentation grayscale_otsu
python tools/bag_replay/h_geometry_offline.py --frames "$run/analysis/h_frames.json" --output "$run/analysis/h_hsv" --segmentation legacy_hsv
python "$report_dir/analyze.py" --run "$run" > "$run/analysis/analyze_stdout.json"
ffmpeg -v error -y -ss 218 -i "$run/replay/dashboard.mp4" -t 16 -an -c:v libx264 -preset fast -crf 21 -threads 2 -pix_fmt yuv420p -movflags +faststart "$run/analysis/slot3_focus.mp4"
ffmpeg -v error -y -ss 285 -i "$run/replay/dashboard.mp4" -t 22 -an -c:v libx264 -preset fast -crf 21 -threads 2 -pix_fmt yuv420p -movflags +faststart "$run/analysis/h_focus.mp4"
ffmpeg -v error -y -ss 172 -i "$run/replay/dashboard.mp4" -t 62 -an -c:v libx264 -preset fast -crf 21 -threads 2 -pix_fmt yuv420p -movflags +faststart "$run/analysis/delivery_focus.mp4"
for f in "$run"/analysis/*focus.mp4; do ffmpeg -v error -i "$f" -f null -; ffprobe -v error -show_entries format=duration,size -of json "$f"; done > "$run/analysis/focus_video_validation.jsonl"