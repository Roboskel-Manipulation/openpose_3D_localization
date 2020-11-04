#ifndef _OPENPOSE_ROS_RECEIVER_H_
#define _OPENPOSE_ROS_RECEIVER_H_

/* OpenPose ROS wrapper headers */
#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

/* ROS headers */
#include <ros/ros.h>

/* TF headers */
#include <tf/transform_listener.h>

/* PCL headers */
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

/* C++ headers */
#include <cmath>
#include <vector>
#include <string>

/* C headers */
#include <assert.h>   // for development phase

/* OpenPose headers */
#include <openpose/pose/poseParameters.hpp>

/* Important definitions */
#define RELIABILITY_THRESHOLD 3.0           // seconds -> float
#define MAX_RETRY 2                         // number of repetitions -> int
#define TF_WAIT 10                          // seconds -> float
#define IMG_PIXEL_WIDTH 672                 // pixels
#define IMG_PIXEL_HEIGHT 376                // pixels
#define UPPER_VARIATION_THRESH 1.15         // how much different is the avg neighborhood value allowed to be in contrast to the point value --> float
#define LOWER_VARIATION_THRESH 0.85         // how much different is the avg neighborhood value allowed to be in contrast to the point value --> float
#define RED_KEYPOINT 255
#define GREEN_KEYPOINT 0
#define BLUE_KEYPOINT 0
#define RED 0
#define GREEN 0
#define BLUE 255

/* Global Variables */
static bool pclMsg, humanListMsg, pointcloudEnable, hand_flag, face_flag;
static ros::Publisher robotFrameCoordsPub, humanReceiverPub, pointcloudDebugPub;
static const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPCL;
static std::vector<int> points_of_interest;
static geometry_msgs::Point point_msg;

/* OpenPose BODY_65 Body Parts Index-to-Name Mapping */
const std::map<int, std::string> POSE_BODY_65_BODY_PARTS
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
    {25, "Background"},
    {26, "LThumb1CMC"},
    {27, "LThumb2Knuckles"},
    {28, "LThumb3IP"},
    {29, "LThumb4FingerTip"},
    {30, "LIndex1Knuckles"},
    {31, "LIndex2PIP"},
    {32, "LIndex3DIP"},
    {33, "LIndex4FingerTip"},
    {34, "LMiddle1Knuckles"},
    {35, "LMiddle2PIP"},
    {36, "LMiddle3DIP"},
    {37, "LMiddle4FingerTip"},
    {38, "LRing1Knuckles"},
    {39, "LRing2PIP"},
    {40, "LRing3DIP"},
    {41, "LRing4FingerTip"},
    {42, "LPinky1Knuckles"},
    {43, "LPinky2PIP"},
    {44, "LPinky3DIP"},
    {45, "LPinky4FingerTip"},
    {46, "RThumb1CMC"},
    {47, "RThumb2Knuckles"},
    {48, "RThumb3IP"},
    {49, "RThumb4FingerTip"},
    {50, "RIndex1Knuckles"},
    {51, "RIndex2PIP"},
    {52, "RIndex3DIP"},
    {53, "RIndex4FingerTip"},
    {54, "RMiddle1Knuckles"},
    {55, "RMiddle2PIP"},
    {56, "RMiddle3DIP"},
    {57, "RMiddle4FingerTip"},
    {58, "RRing1Knuckles"},
    {59, "RRing2PIP"},
    {60, "RRing3DIP"},
    {61, "RRing4FingerTip"},
    {62, "RPinky1Knuckles"},
    {63, "RPinky2PIP"},
    {64, "RPinky3DIP"},
    {65, "RPinky4FingerTip"}
};

// Hand legend:
//     - Thumb:
//         - Carpometacarpal Joints (CMC)
//         - Interphalangeal Joints (IP)
//     - Other fingers:
//         - Knuckles or Metacarpophalangeal Joints (MCP)
//         - PIP (Proximal Interphalangeal Joints)
//         - DIP (Distal Interphalangeal Joints)
//     - All fingers:
//         - Fingertips

// Various functions
std::string getPoseBodyPartMappingBody65(int idx);
std::vector<std::vector<int> > neighborhood_vector(); 
keypoint_3d_matching_msgs::Keypoint3d_list keypointsStructure(std::vector<int> points_of_interest, std::string frame);

#endif