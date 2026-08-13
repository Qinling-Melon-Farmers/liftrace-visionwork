echo ===process===
ps aux | grep 29805 | grep -v grep | head -2
echo ===files===
ls -la ~/board_eval/out_new_int8_v2.mp4 ~/board_eval/perf_new_int8.json 2>/dev/null
echo ===log_tail===
tail -15 ~/board_eval/log_int8_v2.txt
echo ===json_if===
python3 -c "
import json
d = json.load(open('/home/orangepi/board_eval/perf_new_int8.json'))
print('frames', d['sampling']['frames_measured'])
print('infer_ms', d['metrics_ms']['infer_ms'])
print('class_hist', d.get('derived', {}).get('class_hist_total'))
" 2>&1 | head -6
