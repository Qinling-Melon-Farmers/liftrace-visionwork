#!/usr/bin/env bash

# Read-only check for processes that belong to the local ROS/PX4/Gazebo stack.
# Keep process names separate: a regex containing `|` can become a shell
# pipeline if quoting is lost while crossing the Windows -> WSL argv boundary.

set -u

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

process_names=(
  roscore
  rosmaster
  rosout
  roslaunch
  gzserver
  gzclient
  px4
  mavros_node
  rviz
)

found=0
for process_name in "${process_names[@]}"; do
  mapfile -t process_pids < <(pgrep -x "${process_name}" 2>/dev/null || true)
  if ((${#process_pids[@]} > 0)); then
    printf '%s:' "${process_name}"
    printf ' %s' "${process_pids[@]}"
    printf '\n'
    found=1
  fi
done

if ((found != 0)); then
  printf 'Local ROS/Gazebo/PX4/RViz processes are still running.\n' >&2
  exit 1
fi

printf 'No local ROS/Gazebo/PX4/RViz process remains.\n'
