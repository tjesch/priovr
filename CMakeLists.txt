cmake_minimum_required(VERSION 2.8.3)
project(priovr)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  r2_gazebo
  tf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES priovr
CATKIN_DEPENDS 
  geometry_msgs 
  sensor_msgs 
  r2_gazebo
  tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
