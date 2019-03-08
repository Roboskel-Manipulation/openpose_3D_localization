#ifndef _OPENPOSE_ROS_CONTROL_H_
#define _OPENPOSE_ROS_CONTROL_H_

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* MoveIt! headers */
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

/* Global Variables */

/* OpenPose BODY_25 Body Parts Mapping */
const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS
{
    {0,  "Nose"},
    {1,  "Neck"},
    {2,  "RShoulder"},
    {3,  "RElbow"},
    {4,  "RWrist"},
    {5,  "LShoulder"},
    {6,  "LElbow"},
    {7,  "LWrist"},
    {8,  "MidHip"},
    {9,  "RHip"},
    {10, "RKnee"},
    {11, "RAnkle"},
    {12, "LHip"},
    {13, "LKnee"},
    {14, "LAnkle"},
    {15, "REye"},
    {16, "LEye"},
    {17, "REar"},
    {18, "LEar"},
    {19, "LBigToe"},
    {20, "LSmallToe"},
    {21, "LHeel"},
    {22, "RBigToe"},
    {23, "RSmallToe"},
    {24, "RHeel"},
    {25, "Background"}
};
/* OpenPose BODY_25 Body Part Pairs Mapping */
const std::map<unsigned int, unsigned int> POSE_BODY_25_BODY_PART_PAIRS
{
    {1, 8},
    {1, 2},
    {1, 5},
    {2, 3},
    {3, 4},
    {5, 6},
    {6, 7},
    {8, 9},
    {9, 10},
    {10, 11},
    {8, 12},
    {12, 13},
    {13, 14},
    {1, 0},
    {0, 15},
    {15, 17},
    {0, 16},
    {16, 18},
    {2, 17},
    {5, 18},
    {14, 19},
    {19, 20},
    {14, 21},
    {11, 22},
    {22, 23},
    {11, 24}
};

/* Node class */
class OpenPoseROSControl
{
public:
    /* Node functions */
    OpenPoseROSControl();
    /* Callback functions */
    void robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg);
    void robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    std::string robot_frame_coords_str_topic_, robot_frame_coords_msg_topic_, image_frame_, robot_base_link_frame_;
    int queue_size_;
    ros::Subscriber subRobotFrameCoordsStr_, subRobotFrameCoordsMsg_;
};

/* Various functions */
std::string getPoseBodyPartMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx);

#endif