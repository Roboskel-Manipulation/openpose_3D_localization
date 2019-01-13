#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic, pointcloud_topic;
    int queue_size, sync_buffer_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("queue_size", queue_size, 10000);
    nh.param("sync_buffer_size", sync_buffer_size, 10000);

    /* Initialize Global variables */
    initGlobalVars();

    /* Read human list topic, process it and broadcast its keypoints to tf */
    /* Create message filters */
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> subPointcloud(nh, pointcloud_topic, queue_size);
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> subHumanList(nh, human_list_topic, queue_size);
    
    /* Define message filters synchronization policy */
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, openpose_ros_msgs::OpenPoseHumanList> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(sync_buffer_size), subPointcloud, subHumanList);
    sync.registerCallback(boost::bind(&humanListPointcloudSkeletonCallback, _1, _2));

    ros::spin();

    return 0;
}