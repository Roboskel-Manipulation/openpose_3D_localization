#include "openpose_ros_respond.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_respond_node");
    
    OpenPoseROSRespond();

    return 0;
}