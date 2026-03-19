cmake_minimum_required(VERSION 3.0.2)
project(main_command)

find_package(catkin REQUIRED COMPONENTS
  dsr_msgs
  rospy
  std_msgs
  woosh_msgs
)

catkin_package(
  CATKIN_DEPENDS dsr_msgs rospy std_msgs woosh_msgs
)

catkin_install_python(PROGRAMS
  scripts/main_system_operation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
