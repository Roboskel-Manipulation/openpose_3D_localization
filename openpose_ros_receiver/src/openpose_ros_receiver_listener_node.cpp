#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic;
    int queue_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("queue_size", queue_size, 10);

    /* Subscribe to human list topic, listen for keypoints to tf transformations */
    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, listenForSkeleton);

    ros::spin();

    return 0;
}