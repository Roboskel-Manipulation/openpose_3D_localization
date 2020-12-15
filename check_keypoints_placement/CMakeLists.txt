cmake_minimum_required(VERSION 3.0.2)
project(check_keypoints_placement)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  keypoint_3d_matching_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES check_keypoints_placement
#  CATKIN_DEPENDS keypoint_3d_matching_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (check_keypoints_placement src/check_keypoints_placement.cpp)
target_link_libraries (check_keypoints_placement ${catkin_LIBRARIES})

