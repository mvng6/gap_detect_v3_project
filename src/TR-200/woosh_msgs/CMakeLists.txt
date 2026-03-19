cmake_minimum_required(VERSION 3.0.2)
project(woosh_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_service_files(
  FILES
  MoveMobile.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
