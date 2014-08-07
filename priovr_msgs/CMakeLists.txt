cmake_minimum_required(VERSION 2.8.3)
project(priovr_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
)

add_message_files(FILES
  PriovrOrientations.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS 
  geometry_msgs
  message_runtime
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
