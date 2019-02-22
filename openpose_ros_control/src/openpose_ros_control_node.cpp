#include "openpose_ros_control.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_control_node");

    /* Initialize node variables */
    ros::NodeHandle nh;
    std::string robot_frame_coords_topic;
    int queue_size;
    nh.param("openpose_ros_receiver_broadcaster_node/robot_frame_coords_topic", robot_frame_coords_topic, std::string("/openpose_ros_receiver/robot_frame_coords"));
    nh.param("openpose_ros_receiver_broadcaster_node/queue_size", queue_size, 2);

    /* Subscribe to robot frame coordinates topic */
    ros::Subscriber subRobotFrameCoords = nh.subscribe(robot_frame_coords_topic, queue_size, robotFrameCoordsTopicCallback);

    ros::spin();

    return 0;
}