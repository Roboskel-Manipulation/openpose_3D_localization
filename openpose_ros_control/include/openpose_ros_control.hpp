#ifndef _OPENPOSE_ROS_CONTROL_H_
#define _OPENPOSE_ROS_CONTROL_H_

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* Global Variables */

/* OpenPose BODY_25 Body Parts Mapping */
const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS
{
    {0,  "Nose"},
    {1,  "Neck"},
    {2,  "RShoulder"},
    {3,  "RElbow"},
    {4,  "RWrist"},
    {5,  "LShoulder"},
    {6,  "LElbow"},
    {7,  "LWrist"},
    {8,  "MidHip"},
    {9,  "RHip"},
    {10, "RKnee"},
    {11, "RAnkle"},
    {12, "LHip"},
    {13, "LKnee"},
    {14, "LAnkle"},
    {15, "REye"},
    {16, "LEye"},
    {17, "REar"},
    {18, "LEar"},
    {19, "LBigToe"},
    {20, "LSmallToe"},
    {21, "LHeel"},
    {22, "RBigToe"},
    {23, "RSmallToe"},
    {24, "RHeel"},
    {25, "Background"}
};

/* Node functions */

/* Callback functions */
void robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg);
void robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);

/* Various functions */
std::string getPoseBodyPartMappingBody25(unsigned int idx);

#endif