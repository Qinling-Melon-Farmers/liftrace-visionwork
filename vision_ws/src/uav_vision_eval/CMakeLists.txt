cmake_minimum_required(VERSION 3.0.2)
project(uav_vision_eval)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  image_geometry
  message_filters
  message_generation
  rosnode
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
  SimTarget.msg
  SimTargetArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    diagnostic_msgs
    gazebo_msgs
    geometry_msgs
    image_geometry
    message_filters
    message_runtime
    rosnode
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    uav_vision
)

catkin_install_python(PROGRAMS
  scripts/camera_video_recorder.py
  scripts/scene_preview_capture.py
  scripts/competition_key_recorder.py
  scripts/gazebo_camera_pose_stamper.py
  scripts/gazebo_camera_tf.py
  scripts/sim_ground_truth_projector.py
  scripts/vision_metrics_recorder.py
  scripts/vision_metrics_report.py
  scripts/summarize_seed_matrix.py
  scripts/mock_eval_inputs.py
  scripts/eval_pipeline_assertion.py
  scripts/gazebo_truth_assertion.py
  scripts/shadow_contract_assertion.py
  scripts/image_snapshot.py
  scripts/aligning_scene_recorder.py
  scripts/aligning_capture_gate_assertion.py
  scripts/target_memory_freshness_assertion.py
  scripts/target_refiner_association_assertion.py
  scripts/vsim04_dry_run.py
  scripts/vsim04_d50_dry_run.py
  scripts/vsim04_d50_runner.py
  scripts/vsim04_metrics_assertion.py
  scripts/vsim04_navigation_metrics_assertion.py
  scripts/vsim04_trial_recorder.py
  scripts/vsim04_trial_runner.py
  scripts/vsim04_camera_soak.py
  scripts/vsim04_soak_model_spawner.py
  scripts/vsim04_soak_assertion.py
  scripts/vsim04_failure_capture.py
  scripts/vsim04_failure_capture_assertion.py
  scripts/vsim04_failure_capture_manifest_check.py
  scripts/vsim04_repeat_runner.py
  scripts/vsim04_repeat_aggregate.py
  scripts/ros_topology_snapshot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    scripts/vsim04_failure_capture_assertion.py
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  catkin_add_nosetests(
    scripts/vsim04_repeat_runs_assertion.py
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  catkin_add_nosetests(scripts/vsim04_soak_assertion.py)
  catkin_add_nosetests(
    scripts/vsim04_navigation_metrics_assertion.py
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  catkin_add_nosetests(test/test_vsim04_d_matrix.py)
endif()

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Simulation-only contact envelope: preserve contact scoring without making
# the conservative proxy an opaque object in the onboard LiDAR's field of view.
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(uav_contact_envelope SHARED src/contact_envelope_plugin.cpp)
target_include_directories(uav_contact_envelope SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_compile_features(uav_contact_envelope PRIVATE cxx_std_14)
target_link_libraries(uav_contact_envelope ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS uav_contact_envelope
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(contact_envelope_probe test/contact_envelope_probe.cpp)
  target_include_directories(contact_envelope_probe SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS})
  target_compile_definitions(contact_envelope_probe PRIVATE USE_GAZEBO_ODE)
  target_compile_features(contact_envelope_probe PRIVATE cxx_std_14)
  target_link_libraries(contact_envelope_probe gazebo_ode)
endif()
