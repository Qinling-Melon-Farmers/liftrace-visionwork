#!/usr/bin/env bash

# Stop the local SITL process names used by the toudi3 GUI guides.
# This script is intended for a machine dedicated to this local SITL run;
# it does not send commands to a flight controller or actuator.
set +e
for process_name in roslaunch gzserver gzclient px4 mavros_node rosmaster rosout; do
  pkill -TERM -x "${process_name}" 2>/dev/null
done
sleep 2
for process_name in roslaunch gzserver gzclient px4 mavros_node rosmaster rosout; do
  pkill -KILL -x "${process_name}" 2>/dev/null
done
echo "Requested stop for local toudi3 SITL process names."
