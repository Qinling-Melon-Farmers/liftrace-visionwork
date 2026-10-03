#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
mode="${1:-preview}"
exec bash "$script_dir/../../../emit_transcript.sh" "06_high_priority" "$mode"
