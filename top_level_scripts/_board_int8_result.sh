echo ===int8_perf===
python3 -c "
import json
d = json.load(open('/home/orangepi/board_eval/perf_new_int8.json'))
print('frames', d['sampling']['frames_measured'])
print('infer_ms', d['metrics_ms']['infer_ms'])
print('total_ms', d['metrics_ms']['total_ms'])
print('detections:', d.get('derived', {}).get('class_hist_total'))
print('median_det:', d.get('derived', {}).get('median_detections'))
"
echo ===out_files===
ls -la ~/board_eval/out_new_int8.mp4 ~/board_eval/perf_new_int8.json 2>/dev/null
