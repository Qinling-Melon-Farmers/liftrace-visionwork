cd ~/board_eval
nohup python3 board_realtime_rknn_viewer.py --video real_target.mp4 --json perf_new_int8.json --output-video out_new_int8_v2.mp4 --output-width 1280 --stride 1 --warmup 5 --no-window merged_standard_int8_8img.rknn > log_int8_v2.txt 2>&1 &
echo "PID=$!"
sleep 5
head -20 log_int8_v2.txt
