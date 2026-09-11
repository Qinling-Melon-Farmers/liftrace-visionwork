# ROS Noetic integration
Upstream: https://github.com/Livox-SDK/livox_ros_driver2
Revision: 6b9356cadf77084619ba406e6a0eb41163b08039
License: LICENSE.txt (upstream files retained).

Local changes: package.xml is package_ROS1.xml; CMake defaults to ROS1,
locates SDK2 headers/library explicitly and installs config alongside launches.
Build through top_level_scripts/build_competition.sh; do not run upstream
build.sh in this integrated workspace.
Device entry: uav_mission/launch/mid360_driver2.launch, with an explicit
user_config_path for the real MID360/host network. No hardware is started
by the simulation entry.
