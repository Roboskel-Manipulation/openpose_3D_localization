#include "openpose_ros_avoid.hpp"

int main (int argc, char** argv)
{
    /* Initialize ROS node */
    ros::init(argc, argv, "openpose_ros_avoid_node");
    
    OpenPoseROSAvoid();

    return 0;
}