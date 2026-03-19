cmake_minimum_required(VERSION 3.0.2)
project(woosh_slam_amcl)

find_package(catkin REQUIRED COMPONENTS
  amcl
  geometry_msgs
  map_server
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  woosh_robot_py
)

catkin_package(
  CATKIN_DEPENDS amcl geometry_msgs map_server nav_msgs rospy sensor_msgs
                 std_msgs tf2 tf2_ros woosh_robot_py
)

catkin_install_python(PROGRAMS
  scripts/woosh_sensor_bridge.py
  scripts/export_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
