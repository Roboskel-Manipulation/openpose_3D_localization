#include "openpose_ros_receiver.hpp"

void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    for (uint32_t i = 0; i < msg->num_humans; i++)
    {
        for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
            ROS_INFO("%f %f %f", msg->human_list[i].body_key_points_with_prob[j].x, msg->human_list[i].body_key_points_with_prob[j].y, msg->human_list[i].body_key_points_with_prob[j].z);
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "openpose_ros_receiver_node");

    ros::NodeHandle nh;
    std::string human_list_topic;
    int human_list_queue_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("human_list_queue_size", human_list_queue_size, 10);

    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, human_list_queue_size, humanListCallback);

    ros::spin();

    return 0;
}