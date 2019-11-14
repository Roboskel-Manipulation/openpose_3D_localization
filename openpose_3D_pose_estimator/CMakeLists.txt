cmake_minimum_required(VERSION 2.8.3)
project(openpose_3D_pose_estimator)

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
  openpose_3D_pose_estimator_msgs
)
find_package(/usr/local/lib/OpenPose/OpenPose REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros message_filters openpose_ros_msgs openpose_3D_pose_estimator_msgs
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS} 
)

add_executable(openpose_3D_pose_estimator src/openpose_3D_pose_estimation.cpp src/functions.cpp)

add_dependencies(openpose_3D_pose_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(openpose_3D_pose_estimator
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS} 
)
