#include "openpose_ros_control.hpp"

OpenPoseROSControl::OpenPoseROSControl()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_control_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_control_node/queue_size", queue_size_, 2);
    nh_.param("openpose_ros_control/image_frame", image_frame_, std::string("/zed_left_camera_frame"));
    nh_.param("openpose_ros_control/base_link_frame", robot_base_link_frame_, std::string("/base_link"));

    /* Subscribe to robot frame coordinates topic */
    subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsStrTopicCallback, this);
    subRobotFrameCoordsMsg_ = nh_.subscribe(robot_frame_coords_msg_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsMsgTopicCallback, this);

    ros::spin();
}

void OpenPoseROSControl::robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

void OpenPoseROSControl::robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    for (uint8_t i = 0; i < 25; i++)
        ROS_WARN("%s: %f, %f, %f", getPoseBodyPartMappingBody25(i).c_str(), msg->body_key_points_with_prob[i].x, msg->body_key_points_with_prob[i].y, msg->body_key_points_with_prob[i].z);
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}

unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PART_PAIRS.find(idx)->second;
}