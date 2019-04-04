#include "openpose_ros_receiver.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_receiver_broadcaster_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string human_list_topic, pointcloud_topic, pointcloud_topic_debug, robot_frame_coords_str_topic, robot_frame_coords_msg_topic;
    int queue_size;
    nh.param("openpose_ros_receiver_broadcaster_node/human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("openpose_ros_receiver_broadcaster_node/pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("openpose_ros_receiver_broadcaster_node/pointcloud_topic_debug", pointcloud_topic_debug, std::string("/zed/point_cloud/cloud_registered_debug"));
    nh.param("openpose_ros_receiver_broadcaster_node/image_frame", image_frame, std::string("/zed_left_camera_frame"));
    nh.param("openpose_ros_receiver_broadcaster_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh.param("openpose_ros_receiver_broadcaster_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh.param("openpose_ros_receiver_broadcaster_node/queue_size", queue_size, 2);
    nh.param("openpose_ros_receiver_broadcaster_node/logging", logging, false);

    /* Initialize Global Variables */
    tfSubtree = false;
    pclMsg = false; humanListMsg = true;
    /* Advertise robot frame coordinates topic */
    robotFrameCoordsPub = nh.advertise<std_msgs::String>(robot_frame_coords_str_topic, queue_size);
    humanReceiverPub = nh.advertise<openpose_ros_receiver_msgs::OpenPoseReceiverHuman>(robot_frame_coords_msg_topic, queue_size);
    pointcloudDebugPub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(pointcloud_topic_debug, queue_size);

    /* Read human list topic, process it and broadcast its keypoints to tf */
    ros::Subscriber subPointcloud = nh.subscribe(pointcloud_topic, queue_size, pointCloudTopicCallback);
    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, humanListCallback);

    ros::spin();

    return 0;
}
