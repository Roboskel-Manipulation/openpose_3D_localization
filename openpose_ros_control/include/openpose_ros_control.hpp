#ifndef _OPENPOSE_ROS_CONTROL_H_
#define _OPENPOSE_ROS_CONTROL_H_

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* Callback functions */
void robotFrameCoordsTopicCallback(const std_msgs::String::ConstPtr& msg);

#endif