#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
DEST="${1:?usage: package.sh /absolute/output.tar}"
[[ "$DEST" = /* ]] || { echo "Output must be absolute" >&2; exit 2; }
[[ ! -e "$DEST" ]] || { echo "Output already exists" >&2; exit 2; }
git -C "$ROOT" diff --quiet
git -C "$ROOT" diff --cached --quiet
git -C "$ROOT" archive --format=tar -o "$DEST" HEAD \
  patrol_uav_ws-patrol_planner \
  vision_ws/src/CMakeLists.txt vision_ws/src/uav_vision vision_ws/src/uav_high_view vision_ws/src/camera_sdk \
  deployment/competition top_level_scripts/build_competition.sh \
  docs/deployment/competition_integration_20261003 \
  docs/deployment/evening_validation_20261008
printf 'SOURCE_REVISION=%s\n' "$(git -C "$ROOT" rev-parse HEAD)"
printf 'Archive: %s\nAdd runtime_models and a measured field.yaml before field startup.\n' "$DEST"
