#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic, pointcloud_topic, robot_frame_coords_topic;
    int queue_size, sync_buffer_size;
    nh.param("openpose_ros_receiver_broadcaster_node/human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("openpose_ros_receiver_broadcaster_node/pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("openpose_ros_receiver_broadcaster_node/robot_frame_coords_topic", robot_frame_coords_topic, std::string("/openpose_ros_receiver/robot_frame_coords"));
    nh.param("openpose_ros_receiver_broadcaster_node/queue_size", queue_size, 2);
    nh.param("openpose_ros_receiver_broadcaster_node/sync_buffer_size", sync_buffer_size, 2);
    nh.param("openpose_ros_receiver_broadcaster_node/logging", logging, false);

    /* Initialize Global Variables */
    tfSubtree = false;
    // totalNans = 0; callbackVisits = 0;
    pclMsg = false; humanListMsg = true;
    // pPCL = nullptr;
    /* Advertise robot frame coordinates topic */
    robotFrameCoordsPub = nh.advertise<std_msgs::String>(robot_frame_coords_topic, queue_size);

    /* Read human list topic, process it and broadcast its keypoints to tf */
    ros::Subscriber subPointcloud = nh.subscribe(pointcloud_topic, queue_size, pointCloudTopicCallback);
    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, humanListCallback);

    ros::spin();

    return 0;
}

/* OLD */

// int main (int argc, char** argv)
// {
//     /* Initialize ROS node */
//     ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

//     /* Initialize Global Variables */
//     tfSubtree = false; logging = false;
//     // totalNans = 0; callbackVisits = 0;

//     /* Initialize node variables */
//     ros::NodeHandle nh;
//     std::string human_list_topic, pointcloud_topic, robot_frame_coords_topic;
//     int queue_size, sync_buffer_size;
//     nh.param("openpose_ros_receiver_broadcaster_node/human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
//     nh.param("openpose_ros_receiver_broadcaster_node/pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
//     nh.param("openpose_ros_receiver_broadcaster_node/robot_frame_coords_topic", robot_frame_coords_topic, std::string("/openpose_ros_receiver/robot_frame_coords"));
//     nh.param("openpose_ros_receiver_broadcaster_node/queue_size", queue_size, 2);
//     nh.param("openpose_ros_receiver_broadcaster_node/sync_buffer_size", sync_buffer_size, 2);
//     nh.param("openpose_ros_receiver_broadcaster_node/logging", logging, false);

//     /* Advertise robot frame coordinates topic */
//     ros::Publisher robotFrameCoordsPub = nh.advertise<std_msgs::String>(robot_frame_coords_topic, queue_size);

//     /* Read human list topic, process it and broadcast its keypoints to tf */
//     /* Create message filters */
//     message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> subPointcloud(nh, pointcloud_topic, queue_size);
//     message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> subHumanList(nh, human_list_topic, queue_size);
    
//     /* Define message filters synchronization policy */
//     typedef message_filters::sync_policies::ApproximateTime< pcl::PointCloud<pcl::PointXYZ>, openpose_ros_msgs::OpenPoseHumanList > MySyncPolicy;

//     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(sync_buffer_size), subPointcloud, subHumanList);
//     sync.registerCallback(boost::bind(&humanListPointcloudCallback, _1, _2, robotFrameCoordsPub));

//     ros::spin();

//     return 0;
// }