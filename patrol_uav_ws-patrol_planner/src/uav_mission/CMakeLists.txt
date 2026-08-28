cmake_minimum_required(VERSION 3.0.2)
project(uav_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  patrol_control
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
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
  CATKIN_DEPENDS geometry_msgs message_runtime patrol_control rospy sensor_msgs std_msgs tf2_ros uav_vision
)

catkin_install_python(PROGRAMS
  scripts/release_permission_arbiter.py
  scripts/release_commitment.py
  scripts/guarded_servo_proxy.py
  scripts/mock_raw_servo_server.py
  scripts/release_guard_assertion.py
  scripts/visual_delivery_audit.py
  scripts/fixed_drop_evidence_mock.py
  scripts/fixed_three_drop_assertion.py
  scripts/newvision_fixed3_assertion.py
  scripts/external_candidate_manager.py
  scripts/external_candidate_assertion.py
  scripts/legacy_mode_assertion.py
  scripts/coverage_search_manager.py
  scripts/coverage_navigation_assertion.py
  scripts/coverage_r6_assertion.py
  scripts/random_red_cross_spawner.py
  scripts/random_field_spawner.py
  scripts/pose_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/coverage_policy.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
