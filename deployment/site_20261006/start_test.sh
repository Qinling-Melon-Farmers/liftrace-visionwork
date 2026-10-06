#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
exec "${BOARD_PYTHON:-/usr/bin/python3}" "$script_dir/start_test.py" "$@"
