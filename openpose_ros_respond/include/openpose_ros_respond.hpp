#ifndef _OPENPOSE_ROS_RESPOND_H_
#define _OPENPOSE_ROS_RESPOND_H_

/* C++ headers */
#include <assert.h> // for debugging

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* Global Variables */

/* BODY_25 Pose Output Format header */
#include "body_25_pose_output_format.hpp"

/* Gesture class */
class HumanLimbGesture
{
private:
    geometry_msgs::Point bodyPositionInSpace_;
    std::vector<geometry_msgs::Point> limbPositionInSpace_;
public:
    /* Gesture functions */
    /* Class constructors */
    HumanLimbGesture();
    HumanLimbGesture(const geometry_msgs::Point bodyPositionInSpace, const std::vector<geometry_msgs::Point> limbPositionInSpace);
    /* Print limb gesture description */
    void print();
};

/* Node class */
class OpenPoseROSRespond
{
private:
    ros::NodeHandle nh_;
    std::string robot_frame_coords_str_topic_, robot_frame_coords_msg_topic_;
    int queue_size_, human_body_keypoints_;
    double deviation_margin_;
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
    /* Approximate a human body limb's gesture as a series of points */
    HumanLimbGesture ApproximateLimbGesture(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg, const std::vector<geometry_msgs::Point>& limb);
    /* Associate the human body's position in space with a single keypoint */
    geometry_msgs::Point BodyPositionInSpace(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
    /* Approximate a human body limb's position in space with a series of keypoints */
    std::vector<geometry_msgs::Point> LimbPositionInSpace(const std::vector<geometry_msgs::Point>& limb);
};

/* Utility functions */
std::string getPoseBodyPartIndexMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartNameMappingBody25(std::string name);
unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartPredecessorMappingBody25(unsigned int idx);

#endif