#include "openpose_ros_control.hpp"

void robotFrameCoordsTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}