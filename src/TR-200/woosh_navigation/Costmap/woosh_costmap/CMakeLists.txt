cmake_minimum_required(VERSION 3.0.2)
project(woosh_costmap)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  map_server
  nav_msgs
  rospy
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS costmap_2d geometry_msgs map_server nav_msgs rospy
                 sensor_msgs tf2 tf2_ros
)

install(
  DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
