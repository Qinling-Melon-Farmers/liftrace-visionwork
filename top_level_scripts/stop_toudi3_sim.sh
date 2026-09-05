#!/usr/bin/env bash

# Stop the local SITL process names used by the toudi3 GUI guides.
# This script is intended for a machine dedicated to this local SITL run;
# it does not send commands to a flight controller or actuator.
set +e
process_names=(roscore rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rviz)
for process_name in "${process_names[@]}"; do
  pkill -TERM -x "${process_name}" 2>/dev/null
done
sleep 2
for process_name in "${process_names[@]}"; do
  pkill -KILL -x "${process_name}" 2>/dev/null
done

residual_processes=""
for _cleanup_check in 1 2 3; do
  residual_processes=""
  for process_name in "${process_names[@]}"; do
    process_pids="$(pgrep -x "${process_name}" 2>/dev/null | paste -sd, -)"
    if [ -n "${process_pids}" ]; then
      residual_processes="${residual_processes} ${process_name}=${process_pids}"
    fi
  done
  [ -z "${residual_processes}" ] && break
  sleep 1
done

if [ -n "${residual_processes}" ]; then
  echo "Local SITL cleanup failed:${residual_processes}" >&2
  exit 1
fi

echo "Local toudi3 SITL fully stopped; no managed process remains."
