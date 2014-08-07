cmake_minimum_required(VERSION 2.8.3)
project(priovr_description)

find_package(catkin REQUIRED COMPONENTS
  tf
)

catkin_package(
CATKIN_DEPENDS
  joint_state_publisher
  robot_state_publisher
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
