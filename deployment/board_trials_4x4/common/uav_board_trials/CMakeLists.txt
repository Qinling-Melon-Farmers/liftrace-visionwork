cmake_minimum_required(VERSION 3.0.2)
project(uav_board_trials)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/trial_sim_manager.py scripts/trial_sim_gate.py scripts/trial_manager.py scripts/trial_auto_land.py scripts/trial_terminal_hover.py scripts/trial_journal.py scripts/mock_servo.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Optional notebook/SITL video tools are deliberately omitted from board deployment.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/trial_recorder.py")
  catkin_install_python(PROGRAMS scripts/trial_recorder.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
