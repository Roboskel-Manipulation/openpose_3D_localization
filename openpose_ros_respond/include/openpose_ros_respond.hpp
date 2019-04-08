#ifndef _OPENPOSE_ROS_RESPOND_H_
#define _OPENPOSE_ROS_RESPOND_H_

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* Global Variables */

/* BODY_25 Pose Output Format header */
#include "body_25_pose_output_format.hpp"

/* Node class */
class OpenPoseROSRespond
{
private:
    ros::NodeHandle nh_;
    std::string robot_frame_coords_str_topic_, robot_frame_coords_msg_topic_;
    int queue_size_;
    ros::Subscriber subRobotFrameCoordsStr_, subRobotFrameCoordsMsg_;
public:
    /* Node functions */
    /* Class constructor -- ROS node initializer */
    OpenPoseROSRespond();
    /* Callback functions */
    /* Human body keypoint coordinates in the robot's coordinate frams as strings */
    void robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg);
    /* Human body keypoint coordinates in the robot's coordinate frams as regular messages */
    void robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
};

/* Utility functions */
std::string getPoseBodyPartIndexMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartNameMappingBody25(std::string name);
unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartPredecessorMappingBody25(unsigned int idx);

#endif