cmake_minimum_required(VERSION 3.0.2)
project(woosh_slam_cartographer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    tf2
    tf2_ros
)

# launch, config, rviz 디렉터리 설치
install(
  DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
