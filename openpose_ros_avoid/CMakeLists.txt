cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_avoid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  openpose_ros_receiver_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
)
find_package(/usr/local/lib/OpenPose/OpenPose REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp openpose_ros_receiver_msgs moveit_ros_planning_interface moveit_msgs geometry_msgs std_msgs
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS} 
)

add_executable(openpose_ros_avoid_node src/openpose_ros_avoid_node.cpp src/openpose_ros_avoid.cpp src/utilities.cpp)

add_dependencies(openpose_ros_avoid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(openpose_ros_avoid_node
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS} 
)
