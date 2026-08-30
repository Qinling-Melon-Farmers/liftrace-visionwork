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
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    uav_vision
)

catkin_install_python(PROGRAMS
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
  scripts/vsim04_metrics_assertion.py
  scripts/vsim04_trial_recorder.py
  scripts/vsim04_trial_runner.py
  scripts/vsim04_failure_capture.py
  scripts/vsim04_failure_capture_assertion.py
  scripts/vsim04_failure_capture_manifest_check.py
  scripts/vsim04_repeat_runner.py
  scripts/vsim04_repeat_aggregate.py
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
endif()

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
