cmake_minimum_required(VERSION 3.0.2)
project(woosh_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  woosh_control
  woosh_robot_py
  woosh_utils
)

catkin_package(
  CATKIN_DEPENDS rospy woosh_control woosh_robot_py woosh_utils
)

catkin_install_python(PROGRAMS
  scripts/woosh_service_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
