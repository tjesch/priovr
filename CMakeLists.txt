cmake_minimum_required(VERSION 2.8.3)
project(priovr)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  tf
)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS 
  geometry_msgs 
  sensor_msgs 
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
