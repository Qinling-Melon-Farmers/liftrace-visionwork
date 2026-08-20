cmake_minimum_required(VERSION 3.0.2)
project(uav_vision_eval)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  image_geometry
  message_filters
  message_generation
  rospy
  rosgraph
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  uav_vision
)

catkin_python_setup()

add_message_files(
  FILES
  AuxProposal.msg
  AuxProposalArray.msg
  AuxProjection.msg
  AuxProjectionArray.msg
  SimTarget.msg
  SimTargetArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_aux_proposal_policy.py)
  catkin_add_nosetests(test/test_aux_search_policy.py)
endif()

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    image_geometry
    message_filters
    message_runtime
    rospy
    rosgraph
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    uav_vision
)

catkin_install_python(PROGRAMS
  scripts/aux_proposal_assertion.py
  scripts/aux_proposal_mock_inputs.py
  scripts/aux_proposal_provider.py
  scripts/aux_candidate_memory.py
  scripts/aux_blue_static_assertion.py
  scripts/aux_camera_extrinsic_tf.py
  scripts/aux_coarse_projector.py
  scripts/aux_guided_search_manager.py
  scripts/generate_oblique_vehicle_sdf.py
  scripts/oblique_geometry.py
  scripts/oblique_vehicle_camera_assertion.py
  scripts/run_oblique_fixed_matrix.py
  scripts/run_oblique_guided_ab.py
  scripts/run_oblique_search_ab.py
  scripts/aux_projection_assertion.py
  scripts/aux_projection_mock_inputs.py
  scripts/aux_projection_metrics_recorder.py
  scripts/aux_shadow_recorder.py
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
  scripts/timed_shutdown.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_oblique_coverage_shadow.sh
  scripts/run_oblique_guided_ab.sh
  scripts/run_oblique_search_ab.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
