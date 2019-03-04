#include "openpose_ros_control.hpp"

void robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

void robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    for (uint8_t i = 0; i < 25; i++)
        ROS_WARN("%s: %f, %f, %f", getPoseBodyPartMappingBody25(i).c_str(), msg->body_key_points_with_prob[i].x, msg->body_key_points_with_prob[i].y, msg->body_key_points_with_prob[i].z);
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}