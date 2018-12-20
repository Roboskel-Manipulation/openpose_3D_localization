#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic, pointcloud_topic;
    int queue_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("queue_size", queue_size, 10);

    /* Initialize Global variables */
    initGlobalVars();

    /* Read human list topic, process it and broadcast its keypoints to tf */
    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, humanListBroadcastCallback);
    ros::Subscriber subPointcloud = nh.subscribe(pointcloud_topic, queue_size, pointcloudCallback);

    ros::spin();

    return 0;
}