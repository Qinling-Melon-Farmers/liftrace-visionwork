cmake_minimum_required(VERSION 3.0.2)
project(uav_vision)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  image_transport
  cv_bridge
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  TargetDetection.msg
  TargetDetectionArray.msg
  TargetCandidate.msg
  TargetCandidateArray.msg
  DropOffset.msg
  DropReady.msg
  ReleaseEvidence.msg
  AlignmentTargetContext.msg
  ReleaseEvidenceContext.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    diagnostic_msgs
    image_transport
    cv_bridge
    image_geometry
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ---- cross_detector_node ----
add_executable(cross_detector_node
  src/cross_detector_node.cpp
)
target_link_libraries(cross_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(cross_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_alignment_context_policy.py)
  catkin_add_nosetests(test/test_detector_model_contract.py)
  add_rostest(test/cross_geometry_regression.test)
  add_rostest(test/landing_mode_gate.test)
  add_rostest(test/alignment_context.test)
  add_rostest(test/alignment_context_optional.test)
endif()

# ---- circle_detector_node ----
add_executable(circle_detector_node
  src/circle_detector_node.cpp
)
target_link_libraries(circle_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(circle_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# ---- landing_detector_node ----
add_executable(landing_detector_node
  src/landing_detector_node.cpp
)
target_link_libraries(landing_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(landing_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS
  scripts/placeholder_node.py
  scripts/detection_fusion.py
  scripts/detection_fusion_concurrency_assertion.py
  scripts/detect_compat_bridge.py
  scripts/drop_aligner.py
  scripts/target_selection_policy_assertion.py
  scripts/target_memory.py
  scripts/target_refiner.py
  scripts/target_map_projector.py
  scripts/target_detector.py
  scripts/target_detector_rknn.py
  scripts/mock_uav_pose.py
  scripts/mock_image_publisher.py
  scripts/mock_detection_feeder.py
  scripts/mock_target_pair_feeder.py
  scripts/mock_camera_info.py
  scripts/mock_ring_image_publisher.py
  scripts/mock_align_mode_publisher.py
  scripts/phase_d_assertion.py
  scripts/phase_d_map_assertion.py
  scripts/target_memory_physical_assertion.py
  scripts/map_rejection_assertion.py
  scripts/circle_geometry_assertion.py
  scripts/cross_geometry_regression_assertion.py
  scripts/landing_mode_gate_assertion.py
  scripts/drop_aligner_freshness_assertion.py
  scripts/alignment_context_assertion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cross_detector_node circle_detector_node landing_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
