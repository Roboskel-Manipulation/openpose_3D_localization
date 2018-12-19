#include "openpose_ros_receiver.hpp"

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