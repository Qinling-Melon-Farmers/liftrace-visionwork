#!/usr/bin/env bash
# Preflight only: no ROS nodes, simulator, recorder or hardware actions.
set -euo pipefail
LOGS_DIR="${1:?Pass the intended logs directory}"
MIN_GIB="${SIM_MIN_FREE_GIB:-20}"
case "$MIN_GIB" in ''|*[!0-9]*) echo "SIM_MIN_FREE_GIB must be an integer." >&2; exit 64;; esac
if [ -n "${WSL_DISTRO_NAME:-}" ] && [ -z "${SIM_STORAGE_GUARD_PATH:-}" ]; then
  echo "WSL needs SIM_STORAGE_GUARD_PATH set to the host drive containing its VHDX (for example /mnt/f)." >&2
  echo "The free capacity inside ext4 does not measure host disk space." >&2
  exit 64
fi
mkdir -p "$LOGS_DIR"
MIN_BYTES=$((MIN_GIB * 1024 * 1024 * 1024))
for STORAGE_PATH in "$LOGS_DIR" "${SIM_STORAGE_GUARD_PATH:-$LOGS_DIR}"; do
  if [ ! -d "$STORAGE_PATH" ]; then
    echo "Storage path is unavailable: $STORAGE_PATH" >&2; exit 73
  fi
  FREE_BYTES="$(df -B1 --output=avail -- "$STORAGE_PATH" | tail -n 1 | tr -d ' ')"
  case "$FREE_BYTES" in ''|*[!0-9]*) echo "Cannot read free space: $STORAGE_PATH" >&2; exit 73;; esac
  echo "Storage preflight: $STORAGE_PATH available=$FREE_BYTES bytes required=$MIN_BYTES bytes"
  if [ "$FREE_BYTES" -lt "$MIN_BYTES" ]; then
    echo "Insufficient storage for full recording. Free space or select another logs drive before starting." >&2
    exit 73
  fi
done
