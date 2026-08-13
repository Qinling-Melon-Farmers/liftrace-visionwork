ps aux | grep board_realtime | grep -v grep | wc -l
ls -la ~/board_eval/perf_new_fp32.json ~/board_eval/out_new_fp32.mp4 2>/dev/null
echo ===
python3 -c "
import json
d = json.load(open('/home/orangepi/board_eval/perf_new_fp32.json'))
print('frames', d['sampling']['frames_measured'])
print('infer_ms', d['metrics_ms'])
"
