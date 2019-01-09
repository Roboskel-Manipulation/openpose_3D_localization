#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic, pointcloud_topic;
    int queue_size, buffer_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("queue_size", queue_size, 10000);
    nh.param("buffer_size", buffer_size, 10000);

    /* Initialize Global variables */
    initGlobalVars();

    /* Read human list topic, process it and broadcast its keypoints to tf */
    // ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, humanListBroadcastCallback);
    // ros::Subscriber subPointcloud = nh.subscribe(pointcloud_topic, queue_size, pointcloudCallback);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> subPointcloud(nh, pointcloud_topic, queue_size);
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> subHumanList(nh, human_list_topic, queue_size);
    // ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, listenForSkeleton);
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> subSkeletonFromHumanList(nh, human_list_topic, queue_size);
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, openpose_ros_msgs::OpenPoseHumanList, openpose_ros_msgs::OpenPoseHumanList> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(buffer_size), subPointcloud, subHumanList, subSkeletonFromHumanList);
    sync.registerCallback(boost::bind(&humanListPointcloudSkeletonCallback, _1, _2, _3));

    ros::spin();

    return 0;
}