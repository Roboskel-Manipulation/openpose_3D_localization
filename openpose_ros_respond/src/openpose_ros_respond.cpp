#include "openpose_ros_respond.hpp"

/* Class constructor -- ROS node initializer */
OpenPoseROSRespond::OpenPoseROSRespond()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_respond_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_respond_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh_.param("openpose_ros_respond_node/queue_size", queue_size_, 2);

    /* Subscribe to robot frame coordinates topics */
    subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSRespond::robotFrameCoordsStrTopicCallback, this);
    subRobotFrameCoordsMsg_ = nh_.subscribe(robot_frame_coords_msg_topic_, queue_size_, &OpenPoseROSRespond::robotFrameCoordsMsgTopicCallback, this);

    ros::spin();
}

/* Human body keypoint coordinates in the robot's coordinate frams as strings */
void OpenPoseROSRespond::robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

/* Human body keypoint coordinates in the robot's coordinate frams as regular messages */
void OpenPoseROSRespond::robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    ;
}