cmake_minimum_required(VERSION 3.0.2)
project(woosh_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  woosh_robot_py
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy woosh_robot_py
)

catkin_install_python(PROGRAMS
  scripts/battery_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
