#ifndef _OPENPOSE_ROS_CONTROL_H_
#define _OPENPOSE_ROS_CONTROL_H_

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

/* OpenPose ROS wrapper headers */
#include <openpose_ros_receiver_msgs/OpenPoseReceiverHuman.h>
#include <openpose_ros_receiver_msgs/OpenPoseReceiverKeypoint.h>

/* MoveIt! headers */
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

/* Global Variables */

/* BODY_25 Pose Output Format header */
#include "body_25_pose_output_format.hpp"

/* Node class */
class OpenPoseROSControl
{
private:
    ros::NodeHandle nh_;
    std::string robot_frame_coords_str_topic_, robot_frame_coords_msg_topic_, image_sensor_frame_, robot_base_link_frame_;
    int queue_size_, human_body_keypoints_;
    double  geometric_primitive_radius_, basic_limb_safety_radius_,
            default_geometric_primitive_radius_, default_basic_limb_safety_radius_,
            geometric_primitive_radius_adaptation_limit_, basic_limb_safety_radius_adaptation_limit_,
            min_avg_prob_;
    ros::Subscriber subRobotFrameCoordsStr_, subRobotFrameCoordsMsg_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
public:
    /* Node functions */
    /* Class constructor -- ROS node initializer */
    OpenPoseROSControl();
    /* Callback functions */
    /* Human body keypoint coordinates in the robot's coordinate frams as strings */
    void robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg);
    /* Human body keypoint coordinates in the robot's coordinate frams as regular messages */
    void robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
    /* Generate geometric primitives around every detected human body keypoint */
    void generateBasicGeometricPrimitives(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
    /* Generate geometric primitives around every detected human body keypoint, while also tryig to tackle the absence of the non-detected keypoints */
    void generateBasicGeometricPrimitivesPro(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
    /* Generate geometric primitives around between every detected human body keypoints pair recursively */    
    void generateIntermediateGeometricPrimitivesRec(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix);
    /* Generate geometric primitives around between every detected human body keypoints pair iteratively */
    /* work in progress... (TODO) */
    void generateIntermediateGeometricPrimitivesIter(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix);
    /* Adapt the geometric primitive generation parameters e.g. radiuses to be suitable for a given human body message */
    /* work in progress... (TODO) */
    void adaptGeometricPrimitiveGenerationParameters(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg);
};

/* Utility functions */
double distance(geometry_msgs::Point & a, geometry_msgs::Point & b);
std::string getPoseBodyPartIndexMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartNameMappingBody25(std::string name);
unsigned int getPoseBodyPartPairMappingBody25(unsigned int idx);
unsigned int getPoseBodyPartPredecessorMappingBody25(unsigned int idx);

#endif