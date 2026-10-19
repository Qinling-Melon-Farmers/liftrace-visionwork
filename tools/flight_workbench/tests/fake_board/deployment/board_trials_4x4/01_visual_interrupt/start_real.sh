#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
mode="flight"
exec bash "$script_dir/../../../emit_transcript.sh" "01_visual_interrupt" "$mode"
