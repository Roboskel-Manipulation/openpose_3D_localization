#include "openpose_ros_control.hpp"

OpenPoseROSControl::OpenPoseROSControl()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_control_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_control_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh_.param("openpose_ros_control_node/queue_size", queue_size_, 2);
    nh_.param("openpose_ros_control_node/primitive_radius", primitive_radius_, 0.05);
    nh_.param("openpose_ros_control_node/min_avg_prob", min_avg_prob_, 0.25);
    nh_.param("openpose_ros_control/image_frame", image_frame_, std::string("/zed_left_camera_frame"));
    nh_.param("openpose_ros_control/base_link_frame", robot_base_link_frame_, std::string("/base_link"));

    /* Subscribe to robot frame coordinates topic */
    subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsStrTopicCallback, this);
    subRobotFrameCoordsMsg_ = nh_.subscribe(robot_frame_coords_msg_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsMsgTopicCallback, this);

    ros::spin();
}

void OpenPoseROSControl::robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

void OpenPoseROSControl::robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* if a body doesn't have a minimum average probability, then it is possibly a false positive */
    double sumProb = 0.0;
    for (uint8_t i = 0; i < 25; i++)
        sumProb += msg->body_key_points_with_prob[i].prob;
    if (sumProb / msg->num_body_key_points_with_non_zero_prob < min_avg_prob_)
        return;

    /* remove all othe collision obects in our planning scene */
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    for (uint8_t i = 0; i < 25; i++)
    {
        geometry_msgs::Point a;
        a.x = msg->body_key_points_with_prob[i].x; a.y = msg->body_key_points_with_prob[i].y; a.z = msg->body_key_points_with_prob[i].z;

        /* for debugging */
        ROS_WARN("%s: (x, y, z) = (%f, %f, %f), prob = %f", getPoseBodyPartMappingBody25(i).c_str(), a.x, a.y, a.z, msg->body_key_points_with_prob[i].prob);

        /* if there is a valid keypoint (remember in the robot frame (0,0,0) is the base_link of our robot... no way we have a human body keypoint in there) */
        if (a.x && a.y && a.z)
        {
            /* First, add the keypoint itself as an obstacle, using a sphere */
            /* Define a collision object ROS message */
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_link";
            collision_object.id = i;
            /* Define a sphere which will be added to the world */
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(2);
            /* Setting the radius of the sphere. */
            primitive.dimensions[0] = primitive_radius_;
            /* Define a pose for the sphere (specified relative to frame_id) */
            geometry_msgs::Pose sphere_pose;
            /* Setting the position of the sphere */
            sphere_pose.position.x = a.x;
            sphere_pose.position.y = a.y;
            sphere_pose.position.z = a.z;
            /* Add the sphere as collision object */
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(sphere_pose);
            collision_object.operation = collision_object.ADD;
            planning_scene_interface_.applyCollisionObject(collision_object);

            /* Second, look for its pair and, if it exists, connect them with a cylinder, creating a new obstacle between them */
            if (getPoseBodyPartPairMappingBody25(i))
            {
                int j = getPoseBodyPartPairMappingBody25(i);

                geometry_msgs::Point b;
                b.x = msg->body_key_points_with_prob[j].x; b.y = msg->body_key_points_with_prob[j].y; b.z = msg->body_key_points_with_prob[j].z;

                /* if there can be a valid pair */
                if (b.x && b.y && b.z)
                    generatePrimitivesRec(a, b, i+"_"+j);
            }
        }
    }
}

void OpenPoseROSControl::generatePrimitivesRec(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix)
{
    /* the base of our recursion */
    if (distance(a, b) <= 2 * primitive_radius_)
        return;

    /* find the midpoint of the line connecting points a & b */
    geometry_msgs::Point m;
    m.x = (a.x + b.x) / 2; m.y = (a.y + b.y) / 2; m.z = (a.z + b.z) / 2;

    /* create a sphere in space in the position of the midpoint */
    /* Define a collision object ROS message */
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = idPrefix;
    /* Define a sphere which will be added to the world */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(2);
    /* Setting the radius of the sphere. */
    primitive.dimensions[0] = primitive_radius_;
    /* Define a pose for the sphere (specified relative to frame_id) */
    geometry_msgs::Pose sphere_pose;
    /* Setting the position of the sphere */
    sphere_pose.position.x = m.x;
    sphere_pose.position.y = m.y;
    sphere_pose.position.z = m.z;
    /* Add the sphere as collision object */
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    /* recursion branches */
    generatePrimitivesRec(a, m, idPrefix+"_L");   // leftmost half
    generatePrimitivesRec(m, b, idPrefix+"_R");   // rightmost half
}

void OpenPoseROSControl::generatePrimitivesIter(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix)
{
    // TODO
    ;
}