cmake_minimum_required(VERSION 3.0.2)
project(uav_vision_eval)

find_package(catkin REQUIRED COMPONENTS
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
