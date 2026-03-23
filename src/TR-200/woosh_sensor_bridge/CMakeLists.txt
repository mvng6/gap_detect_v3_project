cmake_minimum_required(VERSION 3.0.2)
project(woosh_sensor_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  woosh_robot_py
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs
                 std_msgs tf2 tf2_ros woosh_robot_py
)

catkin_install_python(PROGRAMS
  scripts/woosh_sensor_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
