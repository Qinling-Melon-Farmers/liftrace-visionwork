#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
echo "REAL release service selected; no automatic arming or mission start."
exec bash "$script_dir/start.sh" flight --real-release "$@"
