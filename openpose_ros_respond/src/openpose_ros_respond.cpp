#include "openpose_ros_respond.hpp"

/* Gesture class */

/* Class constructor */
HumanLimbGesture::HumanLimbGesture()
{
    ;
}

/* Class constructor */
HumanLimbGesture::HumanLimbGesture(const geometry_msgs::Point bodyPositionInSpace, const std::vector<geometry_msgs::Point> limbPositionInSpace)
    : bodyPositionInSpace_(bodyPositionInSpace), limbPositionInSpace_(limbPositionInSpace)
{
    ;
}

/* Print limb gesture description */
void HumanLimbGesture::print()
{
    ROS_INFO("Body position in space: (x, y, z) = (%f, %f, %f)", bodyPositionInSpace_.x, bodyPositionInSpace_.y, bodyPositionInSpace_.z);

    ROS_INFO("Limb position in space:");
    for (uint8_t i = 0; i < limbPositionInSpace_.size(); i++)
        ROS_INFO("i = %d, (x, y, z) = (%f, %f, %f)", i, limbPositionInSpace_.at(i).x, limbPositionInSpace_.at(i).y, limbPositionInSpace_.at(i).z);
}

/* Node class */

/* Class constructor -- ROS node initializer */
OpenPoseROSRespond::OpenPoseROSRespond()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_respond_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_respond_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh_.param("openpose_ros_respond_node/queue_size", queue_size_, 2);
    nh_.param("openpose_ros_respond_node/human_body_keypoints", human_body_keypoints_, 25);
    nh_.param("openpose_ros_respond_node/deviation_margin", deviation_margin_, 0.15);

    /* Subscribe to robot frame coordinates topics */
    // subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSRespond::robotFrameCoordsStrTopicCallback, this);
    subRobotFrameCoordsMsg_ = nh_.subscribe(robot_frame_coords_msg_topic_, queue_size_, &OpenPoseROSRespond::robotFrameCoordsMsgTopicCallback, this);

    ros::spin();
}

/* Human body keypoint coordinates in the robot's coordinate frams as strings */
void OpenPoseROSRespond::robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

/* Human body keypoint coordinates in the robot's coordinate frams as regular messages */
void OpenPoseROSRespond::robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* for each limb */
    for (auto const& limbMapping : POSE_BODY_25_LIMBS_NAME_TO_IDX)
    {
        /* limb keypoints pre-processing */
        std::vector<geometry_msgs::Point> limb;

        /* for debugging */
        ROS_INFO("%s", limbMapping.first.c_str());
        for (auto const& keypointIdx : limbMapping.second)
        {
            /* for debugging */
            ROS_INFO("%d", keypointIdx);
            /* gather limb keypoints */
            geometry_msgs::Point tempPoint;
            tempPoint.x = msg->body_key_points_with_prob[keypointIdx].x;
            tempPoint.y = msg->body_key_points_with_prob[keypointIdx].y;
            tempPoint.z = msg->body_key_points_with_prob[keypointIdx].z;
            limb.push_back(tempPoint);
        }

        /* approximate limbs gesture as a series of points */
        HumanLimbGesture humanLimbGesture;
        humanLimbGesture = ApproximateLimbGesture(msg, limb);

        /* for debugging */
        humanLimbGesture.print();

        /* TODO: respond to the limb gesture */
    }
}

/* Approximate a human body limb's gesture as a series of points */
HumanLimbGesture OpenPoseROSRespond::ApproximateLimbGesture(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg, const std::vector<geometry_msgs::Point>& limb)
{
    /* human body's position in space as a single point */
    geometry_msgs::Point bodyPositionInSpace;
    bodyPositionInSpace = BodyPositionInSpace(msg);

    std::vector<geometry_msgs::Point> limbPositionInSpace;
    limbPositionInSpace = LimbPositionInSpace(limb);

    HumanLimbGesture humanLimbGesture(bodyPositionInSpace, limbPositionInSpace);

    return humanLimbGesture;
}

/* Associate the human body's position in space with a single point */
geometry_msgs::Point OpenPoseROSRespond::BodyPositionInSpace(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    geometry_msgs::Point p; 

    for (uint8_t i = 0; i < human_body_keypoints_; i++)
    {
        if (msg->body_key_points_with_prob[i].prob)  // at least one is enough to be a valid keypoint (since (0,0,0) is our robot's base link position)
        {
            p.x += msg->body_key_points_with_prob[i].x; p.y += msg->body_key_points_with_prob[i].y; p.z += msg->body_key_points_with_prob[i].z;
        }
    }

    p.x /= msg->num_body_key_points_with_non_zero_prob; p.y /= msg->num_body_key_points_with_non_zero_prob; p.z /= msg->num_body_key_points_with_non_zero_prob;

    return p;
}

/* Approximate a human body limb's position in space with a series of points */
std::vector<geometry_msgs::Point> OpenPoseROSRespond::LimbPositionInSpace(const std::vector<geometry_msgs::Point>& limb)
{
    /* for debugging */
    assert( limb.size() > 2 );

    std::vector<geometry_msgs::Point> pointsVec;

    for (int i = 1; i < limb.size(); i++)
    {
        geometry_msgs::Point tempPoint;

        tempPoint.x = limb.at(i-1).x - limb.at(i).x;
        tempPoint.y = limb.at(i-1).y - limb.at(i).y;
        tempPoint.z = limb.at(i-1).z - limb.at(i).z;

        pointsVec.push_back(tempPoint);
    }

    return pointsVec;
}