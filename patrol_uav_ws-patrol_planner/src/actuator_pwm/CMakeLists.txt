cmake_minimum_required(VERSION 3.0.2)
project(actuator_pwm)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs patrol_control)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp rospy std_msgs std_srvs patrol_control)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pwm_node1 src/pwm_node1.cpp src/PWMController.cpp)
add_dependencies(pwm_node1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(pwm_node1 ${catkin_LIBRARIES})
install(TARGETS pwm_node1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS init_pwm.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/servo_debug_node.py
  scripts/servo_debug_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_servo_transactions.py)
endif()
