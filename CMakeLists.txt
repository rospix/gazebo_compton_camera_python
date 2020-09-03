cmake_minimum_required(VERSION 2.8.3)
project(compton_camera_python)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

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
  gazebo_rad_msgs
  rad_msgs
  rviz_visual_tools
  visualization_msgs
  mrs_lib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime gazebo_rad_msgs rviz_visual_tools visualization_msgs mrs_lib rad_msgs
   LIBRARIES ComptonPlotter
   DEPENDS Eigen
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(ComptonPlotter src/compton_plotter.cpp)

add_dependencies(ComptonPlotter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(ComptonPlotter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
