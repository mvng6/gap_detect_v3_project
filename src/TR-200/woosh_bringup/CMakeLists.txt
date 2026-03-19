cmake_minimum_required(VERSION 3.0.2)
project(woosh_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
  woosh_msgs
  woosh_robot_py
  woosh_utils
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs visualization_msgs woosh_msgs woosh_robot_py woosh_utils
)

catkin_install_python(PROGRAMS
  scripts/woosh_service_driver.py
  scripts/woosh_rviz_debug.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
