cmake_minimum_required(VERSION 3.0.2)
project(uav_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  patrol_control
  rospy
  std_msgs
  uav_vision
)

add_message_files(
  FILES
  ReleasePermission.msg
  ReleaseResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime patrol_control rospy std_msgs uav_vision
)

catkin_install_python(PROGRAMS
  scripts/release_permission_arbiter.py
  scripts/guarded_servo_proxy.py
  scripts/mock_raw_servo_server.py
  scripts/release_guard_assertion.py
  scripts/visual_delivery_audit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
