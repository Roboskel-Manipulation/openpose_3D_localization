cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_receiver)

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
)
find_package(/usr/local/lib/OpenPose/OpenPose REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros message_filters
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS} 
)

add_executable(openpose_ros_receiver_broadcaster_node src/openpose_ros_receiver_broadcaster_node.cpp src/functions.cpp)

add_dependencies(openpose_ros_receiver_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(openpose_ros_receiver_broadcaster_node
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS} 
)

add_executable(openpose_ros_receiver_listener_node src/openpose_ros_receiver_listener_node.cpp src/functions.cpp)

add_dependencies(openpose_ros_receiver_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(openpose_ros_receiver_listener_node
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS} 
)