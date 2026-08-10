#!/bin/bash
echo "=== perf jsons ==="
ls -la /home/orangepi/board_eval/*.json 2>/dev/null
echo "=== legacy subprocess log tail ==="
tail -30 /home/orangepi/board_eval/log_legacy_rknn.txt 2>/dev/null
echo "=== any python still running ==="
ps aux | grep "[b]oard_" | head -5
echo "=== perf_report ==="
ls -la /home/orangepi/board_eval/perf_report.json 2>/dev/null || echo "no perf_report"
