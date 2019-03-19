#include "openpose_ros_control.hpp"

double distance(geometry_msgs::Point & a, geometry_msgs::Point & b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}

unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PART_PAIRS.find(idx)->second;
}