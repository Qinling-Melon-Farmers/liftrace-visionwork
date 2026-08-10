echo ===perf_run_int8_support===
grep -n "int8\|fp32\|NEW_MODEL\|def run_\|MODELS\|argparse" ~/board_eval/board_perf_run.py | head -25
echo ===new_chain_script===
ls ~/board_eval/*new* ~/board_eval/*video*.py 2>/dev/null
echo ===perf_report_exists===
ls -la ~/board_eval/perf_report.json 2>/dev/null || echo "no perf_report yet"
