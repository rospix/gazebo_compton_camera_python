cmake_minimum_required(VERSION 2.8.3)
project(compton_camera_python)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
