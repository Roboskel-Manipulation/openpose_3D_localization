cmake_minimum_required(VERSION 2.8.3)
project(frame_transpose)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  keypoint_3d_matching_msgs
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frame_transpose
#  CATKIN_DEPENDS geometry_msgs keypoint_3d_matching_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (frame_transpose src/frame_transpose.cpp)
target_link_libraries (frame_transpose ${catkin_LIBRARIES})