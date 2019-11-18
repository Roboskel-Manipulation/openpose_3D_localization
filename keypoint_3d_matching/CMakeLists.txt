cmake_minimum_required(VERSION 2.8.3)
project(keypoint_3d_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  message_filters
  openpose_ros_msgs
  tf
  keypoint_3d_matching_msgs
)
find_package(/usr/local/lib/OpenPose/OpenPose REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros message_filters openpose_ros_msgs   keypoint_3d_matching_msgs
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS} 
)

add_executable(  keypoint_3d_matching src/keypoint_3d_matching.cpp src/functions.cpp)

add_dependencies(  keypoint_3d_matching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(  keypoint_3d_matching
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS} 
)
