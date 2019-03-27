#include "openpose_ros_control.hpp"

/* Class constructor -- ROS node initializer */
OpenPoseROSControl::OpenPoseROSControl()
{
    /* Initialize node variables */
    nh_.param("openpose_ros_control_node/robot_frame_coords_str_topic", robot_frame_coords_str_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_str"));
    nh_.param("openpose_ros_control_node/robot_frame_coords_msg_topic", robot_frame_coords_msg_topic_, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh_.param("openpose_ros_control_node/queue_size", queue_size_, 2);
    nh_.param("openpose_ros_control_node/human_body_keypoints", human_body_keypoints_, 25);
    nh_.param("openpose_ros_control_node/primitive_radius", primitive_radius_, 0.05);
    nh_.param("openpose_ros_control_node/basic_limb_safety_radius", basic_limb_safety_radius_, 0.35);
    nh_.param("openpose_ros_control_node/min_avg_prob", min_avg_prob_, 0.25);
    nh_.param("openpose_ros_control/image_frame", image_frame_, std::string("/zed_left_camera_frame"));
    nh_.param("openpose_ros_control/base_link_frame", robot_base_link_frame_, std::string("/base_link"));

    /* Subscribe to robot frame coordinates topics */
    // subRobotFrameCoordsStr_ = nh_.subscribe(robot_frame_coords_str_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsStrTopicCallback, this);
    subRobotFrameCoordsMsg_ = nh_.subscribe(robot_frame_coords_msg_topic_, queue_size_, &OpenPoseROSControl::robotFrameCoordsMsgTopicCallback, this);

    ros::spin();
}

/* Human body keypoint coordinates in the robot's coordinate frams as strings */
void OpenPoseROSControl::robotFrameCoordsStrTopicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

/* Human body keypoint coordinates in the robot's coordinate frams as regular messages */
void OpenPoseROSControl::robotFrameCoordsMsgTopicCallback(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* if a body doesn't have a minimum average probability, then it is possibly a false positive */
    double sumProb = 0.0;
    for (uint8_t i = 0; i < human_body_keypoints_; i++)
        sumProb += msg->body_key_points_with_prob[i].prob;
    if (sumProb / msg->num_body_key_points_with_non_zero_prob < min_avg_prob_)
        return;

    // generateBasicPrimitives(msg);
    generateBasicPrimitivesPro(msg);
}

/* Generate geometric primitives around every detected human body keypoint */
void OpenPoseROSControl::generateBasicPrimitives(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* remove all other collision obects in our planning scene */
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    for (uint8_t i = 0; i < human_body_keypoints_; i++)
    {
        geometry_msgs::Point a;
        a.x = msg->body_key_points_with_prob[i].x; a.y = msg->body_key_points_with_prob[i].y; a.z = msg->body_key_points_with_prob[i].z;

        // /* for debugging */
        // ROS_WARN("%s: (x, y, z) = (%f, %f, %f), prob = %f", getPoseBodyPartMappingBody25(i).c_str(), a.x, a.y, a.z, msg->body_key_points_with_prob[i].prob);

        /* if there is a valid keypoint (remember in the robot frame (0,0,0) is the base_link of our robot... no way we have a human body keypoint in there) */
        if (a.x && a.y && a.z)
        {
            /* First, add the keypoint A itself as an obstacle, using a sphere */
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

            /* Second, check if keypoint A is paired with another keypoint */
            if (getPoseBodyPartPairMappingBody25(i))
            {
                int j = getPoseBodyPartPairMappingBody25(i);

                geometry_msgs::Point b;
                b.x = msg->body_key_points_with_prob[j].x; b.y = msg->body_key_points_with_prob[j].y; b.z = msg->body_key_points_with_prob[j].z;

                /* if there can be a valid pair (if the pair exists) */
                if (b.x && b.y && b.z)
                {
                    /* Add the keypoint B itself as an obstacle, using a sphere */
                    /* Define a collision object ROS message */
                    moveit_msgs::CollisionObject collision_object;
                    collision_object.header.frame_id = "base_link";
                    collision_object.id = j;
                    /* Define a sphere which will be added to the world */
                    shape_msgs::SolidPrimitive primitive;
                    primitive.type = primitive.SPHERE;
                    primitive.dimensions.resize(2);
                    /* Setting the radius of the sphere. */
                    primitive.dimensions[0] = primitive_radius_;
                    /* Define a pose for the sphere (specified relative to frame_id) */
                    geometry_msgs::Pose sphere_pose;
                    /* Setting the position of the sphere */
                    sphere_pose.position.x = b.x;
                    sphere_pose.position.y = b.y;
                    sphere_pose.position.z = b.z;
                    /* Add the sphere as collision object */
                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(sphere_pose);
                    collision_object.operation = collision_object.ADD;
                    planning_scene_interface_.applyCollisionObject(collision_object);

                    /* Generate the intermediate keypoints of the A-B pair */
                    generateIntermediatePrimitivesRec(a, b, i+"_"+j);
                }
            }
        }
    }
}

/* Generate geometric primitives around every detected human body keypoint, while also tryig to tackle the absence of the non-detected keypoints */
/* work in progress... (TODO) */
void OpenPoseROSControl::generateBasicPrimitivesPro(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* remove all other collision obects in our planning scene */
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    /* for every possible keypoint */
    for (uint8_t i = 0; i < human_body_keypoints_; i++)
    {
        geometry_msgs::Point a;
        a.x = msg->body_key_points_with_prob[i].x; a.y = msg->body_key_points_with_prob[i].y; a.z = msg->body_key_points_with_prob[i].z;

        /* if there is a valid keypoint (remember in the robot frame (0,0,0) is the base_link of our robot... no way we have a human body keypoint in there) */
        if (a.x && a.y && a.z)
        {
            /* Add keypoint A's primitive */
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

            /* check if keypoint A is paired with another keypoint */
            if (getPoseBodyPartPairMappingBody25(i))
            {
                int j = getPoseBodyPartPairMappingBody25(i);

                geometry_msgs::Point b;
                b.x = msg->body_key_points_with_prob[j].x; b.y = msg->body_key_points_with_prob[j].y; b.z = msg->body_key_points_with_prob[j].z;

                /* if there can be a valid pair (if the pair exists) */
                if (b.x && b.y && b.z)
                {
                    /* Add keypoint B's primitive */
                    /* Define a collision object ROS message */
                    moveit_msgs::CollisionObject collision_object;
                    collision_object.header.frame_id = "base_link";
                    collision_object.id = j;
                    /* Define a sphere which will be added to the world */
                    shape_msgs::SolidPrimitive primitive;
                    primitive.type = primitive.SPHERE;
                    primitive.dimensions.resize(2);
                    /* Setting the radius of the sphere. */
                    primitive.dimensions[0] = primitive_radius_;
                    /* Define a pose for the sphere (specified relative to frame_id) */
                    geometry_msgs::Pose sphere_pose;
                    /* Setting the position of the sphere */
                    sphere_pose.position.x = b.x;
                    sphere_pose.position.y = b.y;
                    sphere_pose.position.z = b.z;
                    /* Add the sphere as collision object */
                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(sphere_pose);
                    collision_object.operation = collision_object.ADD;
                    planning_scene_interface_.applyCollisionObject(collision_object);

                    /* Generate the intermediate keypoints of the A-B pair */
                    generateIntermediatePrimitivesRec(a, b, i+"_"+j);
                }
                else
                {
                    /* check if the non-detected pair keypoint B is an upper body keypoint */
                    if (std::find(POSE_BODY_25_UPPER_BODY_PARTS_IDX.begin(), POSE_BODY_25_UPPER_BODY_PARTS_IDX.end(), i) != POSE_BODY_25_UPPER_BODY_PARTS_IDX.end())
                    {
                        /* for debugging */
                        ROS_WARN("kp %s not detected", getPoseBodyPartMappingBody25(j).c_str());
                        
                        /* TODO: Make an assumption about keypoint B's position using a relatively big sphere */
                        
                    }
                    else
                        continue;
                }
            }
        }
        else
        {
            /* check if the non-detected keypoint A is an upper body keypoint */
            if (std::find(POSE_BODY_25_UPPER_BODY_PARTS_IDX.begin(), POSE_BODY_25_UPPER_BODY_PARTS_IDX.end(), i) != POSE_BODY_25_UPPER_BODY_PARTS_IDX.end())
            {
                /* for debugging */
                ROS_WARN("kp %s not detected", getPoseBodyPartMappingBody25(i).c_str());
                
                /* check if keypoint A is paired with another keypoint */
                if (getPoseBodyPartPairMappingBody25(i))
                {
                    int j = getPoseBodyPartPairMappingBody25(i);

                    geometry_msgs::Point b;
                    b.x = msg->body_key_points_with_prob[j].x; b.y = msg->body_key_points_with_prob[j].y; b.z = msg->body_key_points_with_prob[j].z;

                    /* if there can be a valid pair (if the pair exists) */
                    if (b.x && b.y && b.z)
                    {
                        /* for debugging */
                        ROS_INFO("kp %s (pair of kp %s) detected", getPoseBodyPartMappingBody25(j).c_str(), getPoseBodyPartMappingBody25(i).c_str());

                        /* Add keypoint B's primitive */
                        /* Define a collision object ROS message */
                        moveit_msgs::CollisionObject collision_object;
                        collision_object.header.frame_id = "base_link";
                        collision_object.id = j;
                        /* Define a sphere which will be added to the world */
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.SPHERE;
                        primitive.dimensions.resize(2);
                        /* Setting the radius of the sphere. */
                        primitive.dimensions[0] = primitive_radius_;
                        /* Define a pose for the sphere (specified relative to frame_id) */
                        geometry_msgs::Pose sphere_pose;
                        /* Setting the position of the sphere */
                        sphere_pose.position.x = b.x;
                        sphere_pose.position.y = b.y;
                        sphere_pose.position.z = b.z;
                        /* Add the sphere as collision object */
                        collision_object.primitives.push_back(primitive);
                        collision_object.primitive_poses.push_back(sphere_pose);
                        collision_object.operation = collision_object.ADD;
                        planning_scene_interface_.applyCollisionObject(collision_object);
                        
                        /* TODO: Make an assumption about keypoint A's position using a relatively big sphere */

                        /* Generate the intermediate keypoints of the A-B pair */
                        generateIntermediatePrimitivesRec(a, b, i+"_"+j);
                    }
                    else
                    {
                        /* check if the non-detected pair keypoint B is an upper body keypoint */
                        if (std::find(POSE_BODY_25_UPPER_BODY_PARTS_IDX.begin(), POSE_BODY_25_UPPER_BODY_PARTS_IDX.end(), i) != POSE_BODY_25_UPPER_BODY_PARTS_IDX.end())
                        {
                            /* for debugging */
                            ROS_WARN("kp %s (pair of kp %s) not detected", getPoseBodyPartMappingBody25(j).c_str(), getPoseBodyPartMappingBody25(i).c_str());
                            
                            /* TODO: Make an assumption about A-B pair's position using a relatively big sphere */

                        }
                        else
                            continue;
                    }
                }
            }
            else
                continue;
        }
    }
}

/* Generate geometric primitives around between every detected human body keypoints pair recursively */
void OpenPoseROSControl::generateIntermediatePrimitivesRec(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix)
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
    generateIntermediatePrimitivesRec(a, m, idPrefix+"_L");   // leftmost half
    generateIntermediatePrimitivesRec(m, b, idPrefix+"_R");   // rightmost half
}

/* Generate geometric primitives around between every detected human body keypoints pair iteratively */
/* work in progress... (TODO) */
void OpenPoseROSControl::generateIntermediatePrimitivesIter(geometry_msgs::Point a, geometry_msgs::Point b, std::string idPrefix)
{
    /* check if there is enough space for another primitive */
    if (distance(a, b) <= 2 * primitive_radius_)
        return;

    /* find the distance d between points A & B */
    double d = distance(a, b);
    geometry_msgs::Point n;
    int segment = 0, numerator = d / primitive_radius_ - 1, denominator = d / primitive_radius_;

    /* starting from B, iteratively, take portions of the line between A & B */
    while (numerator > 0)
    {
        // /* for debugging */
        // ROS_INFO("distance: %f, primitive_radius_: %f, segment: %d, numerator: %d, denominator: %d", d, primitive_radius_, segment, numerator, denominator);
        // ROS_INFO("a.x = %f, b.x = %f -- a.y = %f, b.y = %f -- a.z = %f, b.z = %f", a.x, b.x, a.y, b.y, a.z, b.z);
        // ROS_INFO("n.x = %f, n.y = %f, n.z = %f", n.x, n.y, n.z);

        n.x = (a.x + b.x) * numerator / denominator; n.y = (a.y + b.y) * numerator / denominator; n.z = (a.z + b.z) * numerator / denominator;

        /* create a sphere in space in the position of the midpoint */
        /* Define a collision object ROS message */
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = idPrefix + "_" + std::to_string(segment);
        /* Define a sphere which will be added to the world */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(2);
        /* Setting the radius of the sphere. */
        primitive.dimensions[0] = primitive_radius_;
        /* Define a pose for the sphere (specified relative to frame_id) */
        geometry_msgs::Pose sphere_pose;
        /* Setting the position of the sphere */
        sphere_pose.position.x = n.x;
        sphere_pose.position.y = n.y;
        sphere_pose.position.z = n.z;
        /* Add the sphere as collision object */
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(sphere_pose);
        collision_object.operation = collision_object.ADD;
        planning_scene_interface_.applyCollisionObject(collision_object);

        segment++;
        numerator -= 2;
    }
}