cmake_minimum_required(VERSION 2.8.3)
project(aihitplt_ctrl)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aihitplt_ctrl
#  CATKIN_DEPENDS roscpp rosmsg rospy
#  DEPENDS system_lib
)


