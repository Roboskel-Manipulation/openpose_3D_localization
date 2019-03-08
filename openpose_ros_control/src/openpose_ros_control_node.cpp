#include "openpose_ros_control.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_control_node");
    
    OpenPoseROSControl();

    return 0;
}