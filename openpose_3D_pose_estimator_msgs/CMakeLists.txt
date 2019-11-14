cmake_minimum_required(VERSION 2.8.3)
project(openpose_3D_pose_estimator_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

add_message_files(
  FILES
  Openpose_3D.msg
  Openpose_3D_v.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)