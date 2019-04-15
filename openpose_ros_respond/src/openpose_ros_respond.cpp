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

/* Node class */

/* Class constructor -- ROS node initializer */
OpenPoseROSRespond::OpenPoseROSRespond()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_respond_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_respond_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh_.param("openpose_ros_respond_node/queue_size", queue_size_, 2);
    nh_.param("openpose_ros_respond_node/human_body_keypoints", human_body_keypoints_, 25);

    /* Subscribe to robot frame coordinates topics */
    subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSRespond::robotFrameCoordsStrTopicCallback, this);
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
    ;
}

/* Approximate a human body limb's gesture as a series of points */
HumanLimbGesture OpenPoseROSRespond::ApproximateLimbGesture(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* human body's position in space as a single point */
    geometry_msgs::Point bodyPositionInSpace;

    bodyPositionInSpace = BodyPositionInSpace(msg);

    /* limb keypoints pre-processing */
    std::vector<geometry_msgs::Point> limb;

    /* TODO: gather limb keypoints */

    std::vector<geometry_msgs::Point> limbPositionInSpace;

    limbPositionInSpace = LimbPositionInSpace(limb);

    HumanLimbGesture humanLimbGesture(bodyPositionInSpace, limbPositionInSpace);

    return humanLimbGesture;
}

/* Associate the human body's position in space with a single point */
geometry_msgs::Point OpenPoseROSRespond::BodyPositionInSpace(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    geometry_msgs::Point p; 

    for (int i = 0; i < human_body_keypoints_; i++)
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