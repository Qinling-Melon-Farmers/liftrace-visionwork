#!/usr/bin/env bash
set -euo pipefail
review_dir=${BASH_SOURCE[0]%/*}
bash "$review_dir/prepare_offline.sh"
bash "$review_dir/run_cpp.sh"
bash "$review_dir/run_health.sh"
