#!/bin/bash
cd /home/orangepi/board_eval
rm -f perf_report.json
nohup python3 board_perf_run.py --video /home/orangepi/board_eval/real_target.mp4 --max-frames-pt 300 > /home/orangepi/board_eval/perf_run.log 2>&1 &
echo "LAUNCHED pid=$!"
sleep 8
echo "--- early log ---"
head -20 /home/orangepi/board_eval/perf_run.log
ps aux | grep -c "[b]oard_perf_run"
