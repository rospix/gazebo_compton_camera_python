cmake_minimum_required(VERSION 3.1.2)
project(gazebo_compton_camera_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_rad_msgs
  rad_msgs
)

catkin_package(
   CATKIN_DEPENDS rospy gazebo_rad_msgs rad_msgs
)
