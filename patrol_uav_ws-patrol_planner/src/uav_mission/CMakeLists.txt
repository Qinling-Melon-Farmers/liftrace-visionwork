cmake_minimum_required(VERSION 3.0.2)
project(uav_mission)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  mavros_msgs
  nav_msgs
  patrol_control
  plan_manage
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  uav_vision
)

catkin_python_setup()

add_message_files(
  FILES
  NavigationDecision.msg
  NavigationResult.msg
  ReleasePermission.msg
  ReleaseResult.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    message_runtime
    mavros_msgs
    nav_msgs
    patrol_control
    plan_manage
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    uav_vision
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
  scripts/navigation_mission_manager.py
  scripts/navigation_mission_start_gate.py
  scripts/navigation_planner_bridge.py
  scripts/navigation_vcl06_assertion.py
  scripts/target_search_manager_py.py
  scripts/navigation_visual_delivery_adapter.py
  scripts/profile_candidate_selector.py
  scripts/planner_anchor_spawner.py
  scripts/navigation_random_field_assertion.py
  scripts/gazebo_contact_monitor.py
  scripts/navigation_random_field_preflight.py
  scripts/navigation_ab_observer.py
  scripts/navigation_ab_compare.py
  scripts/target_search_delivery_assertion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/coverage_policy.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_contract_profile.py)
  catkin_add_nosetests(test/test_coverage_route.py)
  catkin_add_nosetests(test/test_mission_core.py)
  catkin_add_nosetests(test/test_mission_runtime.py)
  catkin_add_nosetests(test/test_navigation_manager_contract.py)
  catkin_add_nosetests(test/test_mission_start_gate.py)
  catkin_add_nosetests(test/test_planner_execution.py)
  catkin_add_nosetests(test/test_planner_mission_contract.py)
  catkin_add_nosetests(test/test_navigation_planner_bridge_contract.py)
  catkin_add_nosetests(test/test_navigation_vcl06_assertion.py)
  catkin_add_nosetests(test/test_visual_delivery_audit_policy.py)
endif()
