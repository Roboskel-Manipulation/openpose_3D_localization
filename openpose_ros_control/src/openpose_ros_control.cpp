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
    default_primitive_radius_ = primitive_radius_;
    nh_.param("openpose_ros_control_node/basic_limb_safety_radius", basic_limb_safety_radius_, 0.35);
    default_basic_limb_safety_radius_ = basic_limb_safety_radius_;
    nh_.param("openpose_ros_control_node/primitive_radius_adaptation_limit", primitive_radius_adaptation_limit_, 0.25);
    nh_.param("openpose_ros_control_node/basic_limb_safety_radius_adaptation_limit", basic_limb_safety_radius_adaptation_limit_, 1.25);
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

    // double beginSec = ros::Time::now().toSec();
    // adaptPrimitiveGenerationParameters(msg);
    // double endSec = ros::Time::now().toSec();
    // ROS_INFO("adaptPrimitiveGenerationParameters duration: %f", endSec - beginSec);

    // generateBasicPrimitives(msg);

    // beginSec = ros::Time::now().toSec();
    generateBasicPrimitivesPro(msg);
    // endSec = ros::Time::now().toSec();
    // ROS_INFO("generateBasicPrimitivesPro duration: %f", endSec - beginSec);
}

/* Generate geometric primitives around every detected human body keypoint */
void OpenPoseROSControl::generateBasicPrimitives(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* remove all other collision obects in our planning scene */
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    for (uint8_t i = 0; i < human_body_keypoints_; i++)
    {
        /* no need to deal with, knees, ankles, heels, toes and background in our application */
        if (i == 10 || i == 13)     // knees (11, 12) and ankles (13, 14)
        {
            i++;
            continue;
        }
        if (i > 18)                 // heels, toes and background
            break;

        geometry_msgs::Point a;
        a.x = msg->body_key_points_with_prob[i].x; a.y = msg->body_key_points_with_prob[i].y; a.z = msg->body_key_points_with_prob[i].z;

        // /* for debugging */
        // ROS_WARN("%s: (x, y, z) = (%f, %f, %f), prob = %f", getPoseBodyPartIndexMappingBody25(i).c_str(), a.x, a.y, a.z, msg->body_key_points_with_prob[i].prob);

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
void OpenPoseROSControl::generateBasicPrimitivesPro(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* remove all other collision obects in our planning scene */
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    /* for every possible keypoint */
    for (uint8_t i = 0; i < human_body_keypoints_; i++)
    {
        /* no need to deal with, knees, ankles, heels, toes and background in our application */
        if (i == 10 || i == 13)     // knees (11, 12) and ankles (13, 14)
        {
            i++;
            continue;
        }
        if (i > 18)                 // heels, toes and background
            break;

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
            primitive.dimensions[0] = ( i == 4 || i == 7 ? 2*primitive_radius_ : primitive_radius_ ); // wrists need a bigger sphere to cover the hand
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
                    primitive.dimensions[0] = ( j == 4 || j == 7 ? 2*primitive_radius_ : primitive_radius_ ); // wrists need a bigger sphere to cover the hand
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
                    /* we moved the checking of the base condition of our recursion here in order to avoid unnecessary function calls */
                    if (distance(a, b) > 2 * primitive_radius_)
                    {
                        if ( (i == 1 && j == 8) || (i == 8 && j == 1))
                        {
                            /* find the middle of the torso */
                            geometry_msgs::Point m;
                            m.x = (a.x + b.x) / 2; m.y = (a.y + b.y) / 2; m.z = (a.z + b.z) / 2;

                            /* create a sphere in space in the position of the midpoint */
                            /* Define a collision object ROS message */
                            moveit_msgs::CollisionObject collision_object;
                            collision_object.header.frame_id = "base_link";
                            collision_object.id = i+"_"+j;
                            /* Define a sphere which will be added to the world */
                            shape_msgs::SolidPrimitive primitive;
                            primitive.type = primitive.SPHERE;
                            primitive.dimensions.resize(2);
                            /* Setting the radius of the sphere. */
                            primitive.dimensions[0] = basic_limb_safety_radius_;
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
                        }
                        else
                        {
                            // double beginSec = ros::Time::now().toSec();
                            generateIntermediatePrimitivesRec(a, b, i+"_"+j);
                            // double endSec = ros::Time::now().toSec();
                            // ROS_INFO("generateIntermediatePrimitivesRec duration: %f", endSec - beginSec);
                        }
                    }
                }
                else
                {
                    /* check if the non-detected pair keypoint B is an upper body keypoint */
                    if (std::find(POSE_BODY_25_UPPER_BODY_PARTS_IDX.begin(), POSE_BODY_25_UPPER_BODY_PARTS_IDX.end(), i) != POSE_BODY_25_UPPER_BODY_PARTS_IDX.end())
                    {
                        // /* for debugging */
                        // ROS_WARN("kp %s not detected", getPoseBodyPartIndexMappingBody25(j).c_str());
                        
                        /* Make an assumption about keypoint B's position using a relatively big sphere around keypoint A's position */
                        /* Define a collision object ROS message */
                        moveit_msgs::CollisionObject collision_object;
                        collision_object.header.frame_id = "base_link";
                        collision_object.id = j+"_assumption";
                        /* Define a sphere which will be added to the world */
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.SPHERE;
                        primitive.dimensions.resize(2);
                        /* Setting the radius of the sphere. */
                        primitive.dimensions[0] = basic_limb_safety_radius_;
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
                // /* for debugging */
                // ROS_WARN("kp %s not detected", getPoseBodyPartIndexMappingBody25(i).c_str());
                
                /* check if keypoint A is paired with another keypoint */
                if (getPoseBodyPartPairMappingBody25(i))
                {
                    int j = getPoseBodyPartPairMappingBody25(i);

                    geometry_msgs::Point b;
                    b.x = msg->body_key_points_with_prob[j].x; b.y = msg->body_key_points_with_prob[j].y; b.z = msg->body_key_points_with_prob[j].z;

                    /* if there can be a valid pair (if the pair exists) */
                    if (b.x && b.y && b.z)
                    {
                        // /* for debugging */
                        // ROS_INFO("kp %s (pair of kp %s) detected", getPoseBodyPartIndexMappingBody25(j).c_str(), getPoseBodyPartIndexMappingBody25(i).c_str());

                        /* Add keypoint B's primitive, while also making an assumption about keypoint A's position using a relatively big sphere around keypoint B's position */
                        /* Define a collision object ROS message */
                        moveit_msgs::CollisionObject collision_object;
                        collision_object.header.frame_id = "base_link";
                        collision_object.id = i+"_assumption";
                        /* Define a sphere which will be added to the world */
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.SPHERE;
                        primitive.dimensions.resize(2);
                        /* Setting the radius of the sphere. */
                        primitive.dimensions[0] = basic_limb_safety_radius_;
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
                    }
                    else
                    {
                        /* check if the non-detected pair keypoint B is an upper body keypoint */
                        if (std::find(POSE_BODY_25_UPPER_BODY_PARTS_IDX.begin(), POSE_BODY_25_UPPER_BODY_PARTS_IDX.end(), i) != POSE_BODY_25_UPPER_BODY_PARTS_IDX.end())
                        {
                            // /* for debugging */
                            // ROS_WARN("kp %s (pair of kp %s) not detected", getPoseBodyPartIndexMappingBody25(j).c_str(), getPoseBodyPartIndexMappingBody25(i).c_str());
                            
                            /* Check if one of the non-detected pair's keypoints has a predecessor-keypoint */
                            /* A pair's "predecessor" is the keypoint through which one of the pair's keypoints is immediately connected with the rest of the human body */
                            /* First, check for the keypoint A, the keypoint which is closer to the body's core */
                            if (POSE_BODY_25_UPPER_BODY_PART_PAIRS_PREDECESSORS.find(i) != POSE_BODY_25_UPPER_BODY_PART_PAIRS_PREDECESSORS.end())
                            {
                                int k = getPoseBodyPartPredecessorMappingBody25(i);

                                geometry_msgs::Point c;
                                c.x = msg->body_key_points_with_prob[k].x; c.y = msg->body_key_points_with_prob[k].y; c.z = msg->body_key_points_with_prob[k].z;

                                /* if there can be a valid predecessor (if the predecessor exists) */
                                if (c.x && c.y && c.z)
                                {
                                    /* Make an assumption about A-B pair's position using a relatively big sphere around the pair's predecessor position */
                                    /* Define a collision object ROS message */
                                    moveit_msgs::CollisionObject collision_object;
                                    collision_object.header.frame_id = "base_link";
                                    collision_object.id = std::to_string(i)+"-"+std::to_string(j)+"_pair_assumption";
                                    /* Define a sphere which will be added to the world */
                                    shape_msgs::SolidPrimitive primitive;
                                    primitive.type = primitive.SPHERE;
                                    primitive.dimensions.resize(2);
                                    /* Setting the radius of the sphere. */
                                    primitive.dimensions[0] = 2*basic_limb_safety_radius_;
                                    /* Define a pose for the sphere (specified relative to frame_id) */
                                    geometry_msgs::Pose sphere_pose;
                                    /* Setting the position of the sphere */
                                    sphere_pose.position.x = c.x;
                                    sphere_pose.position.y = c.y;
                                    sphere_pose.position.z = c.z;
                                    /* Add the sphere as collision object */
                                    collision_object.primitives.push_back(primitive);
                                    collision_object.primitive_poses.push_back(sphere_pose);
                                    collision_object.operation = collision_object.ADD;
                                    planning_scene_interface_.applyCollisionObject(collision_object);
                                }
                            }
                            /* Second, if needed, check for the keypoint B */
                            else if (POSE_BODY_25_UPPER_BODY_PART_PAIRS_PREDECESSORS.find(j) != POSE_BODY_25_UPPER_BODY_PART_PAIRS_PREDECESSORS.end())
                            {
                                int k = getPoseBodyPartPredecessorMappingBody25(j);

                                geometry_msgs::Point c;
                                c.x = msg->body_key_points_with_prob[k].x; c.y = msg->body_key_points_with_prob[k].y; c.z = msg->body_key_points_with_prob[k].z;

                                /* if there can be a valid predecessor (if the predecessor exists) */
                                if (c.x && c.y && c.z)
                                {
                                    /* Make an assumption about A-B pair's position using a relatively big sphere around the pair's predecessor position */
                                    /* Define a collision object ROS message */
                                    moveit_msgs::CollisionObject collision_object;
                                    collision_object.header.frame_id = "base_link";
                                    collision_object.id = std::to_string(i)+"-"+std::to_string(j)+"_pair_assumption";
                                    /* Define a sphere which will be added to the world */
                                    shape_msgs::SolidPrimitive primitive;
                                    primitive.type = primitive.SPHERE;
                                    primitive.dimensions.resize(2);
                                    /* Setting the radius of the sphere. */
                                    primitive.dimensions[0] = 2*basic_limb_safety_radius_;
                                    /* Define a pose for the sphere (specified relative to frame_id) */
                                    geometry_msgs::Pose sphere_pose;
                                    /* Setting the position of the sphere */
                                    sphere_pose.position.x = c.x;
                                    sphere_pose.position.y = c.y;
                                    sphere_pose.position.z = c.z;
                                    /* Add the sphere as collision object */
                                    collision_object.primitives.push_back(primitive);
                                    collision_object.primitive_poses.push_back(sphere_pose);
                                    collision_object.operation = collision_object.ADD;
                                    planning_scene_interface_.applyCollisionObject(collision_object);
                                }
                            }
                            else
                                continue;
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
    // /* the base of our recursion */
    // if (distance(a, b) <= 2 * primitive_radius_)
    //     return;

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
    /* we moved the checking of the base condition of our recursion here in order to avoid unnecessary function calls */
    if (distance(a, m) > 2 * primitive_radius_)
        generateIntermediatePrimitivesRec(a, m, idPrefix+"_L");   // leftmost half
    if (distance(m, b) > 2 * primitive_radius_)
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

/* Adapt the geometric primitive generation parameters e.g. radiuses to be suitable for a given human body message */
/* work in progress... (TODO) */
void OpenPoseROSControl::adaptPrimitiveGenerationParameters(const openpose_ros_receiver_msgs::OpenPoseReceiverHuman::ConstPtr& msg)
{
    /* ideally:
        * primitive_radius = abs( d(LHip, RHip) - d(LShoulder, RShoulder) ) / 2
        * basic_limb_safety_radius = max( max( d(LShoulder, LElbow), d(RShoulder, RElbow) ), max( d(LElbow, LWrist), d(RElbow, RWrist) ) ) */
    /* for better performance we need to minimize the accesses to our maps */
    geometry_msgs::Point LShoulder, RShoulder, LElbow, RElbow, LWrist, RWrist, LHip, RHip;
    bool hasLShoulder = false, hasRShoulder = false, hasLElbow = false, hasRElbow = false, hasLWrist = false, hasRWrist = false, hasLHip = false, hasRHip = false;
    uint8_t LShoulder_idx, RShoulder_idx, LElbow_idx, RElbow_idx, LWrist_idx, RWrist_idx, LHip_idx, RHip_idx;

    // LShoulder_idx = getPoseBodyPartNameMappingBody25("LShoulder"); RShoulder_idx = getPoseBodyPartNameMappingBody25("RShoulder");
    // LElbow_idx = getPoseBodyPartNameMappingBody25("LElbow"); RElbow_idx = getPoseBodyPartNameMappingBody25("RElbow");
    // LWrist_idx = getPoseBodyPartNameMappingBody25("LWrist"); RWrist_idx = getPoseBodyPartNameMappingBody25("RWrist");
    // LHip_idx = getPoseBodyPartNameMappingBody25("LHip"); RHip_idx = getPoseBodyPartNameMappingBody25("RHip");
    /* better for performance this way: */
    LShoulder_idx = 5; LElbow_idx = 6; LWrist_idx = 7; LHip_idx = 12;
    RShoulder_idx = 2; RElbow_idx = 3; RWrist_idx = 4; RHip_idx = 9;

    LShoulder.x = msg->body_key_points_with_prob[LShoulder_idx].x; LShoulder.y = msg->body_key_points_with_prob[LShoulder_idx].y; LShoulder.z = msg->body_key_points_with_prob[LShoulder_idx].z;
    RShoulder.x = msg->body_key_points_with_prob[RShoulder_idx].x; RShoulder.y = msg->body_key_points_with_prob[RShoulder_idx].y; RShoulder.z = msg->body_key_points_with_prob[RShoulder_idx].z;
    LElbow.x = msg->body_key_points_with_prob[LElbow_idx].x; LElbow.y = msg->body_key_points_with_prob[LElbow_idx].y; LElbow.z = msg->body_key_points_with_prob[LElbow_idx].z;
    RElbow.x = msg->body_key_points_with_prob[RElbow_idx].x; RElbow.y = msg->body_key_points_with_prob[RElbow_idx].y; RElbow.z = msg->body_key_points_with_prob[RElbow_idx].z;
    LWrist.x = msg->body_key_points_with_prob[LWrist_idx].x; LWrist.y = msg->body_key_points_with_prob[LWrist_idx].y; LWrist.z = msg->body_key_points_with_prob[LWrist_idx].z;
    RWrist.x = msg->body_key_points_with_prob[RWrist_idx].x; RWrist.y = msg->body_key_points_with_prob[RWrist_idx].y; RWrist.z = msg->body_key_points_with_prob[RWrist_idx].z;
    LHip.x = msg->body_key_points_with_prob[LHip_idx].x; LHip.y = msg->body_key_points_with_prob[LHip_idx].y; LHip.z = msg->body_key_points_with_prob[LHip_idx].z;
    RHip.x = msg->body_key_points_with_prob[RHip_idx].x; RHip.y = msg->body_key_points_with_prob[RHip_idx].y; RHip.z = msg->body_key_points_with_prob[RHip_idx].z;

    /* find which keypoints exist in the human body message at hand */
    if (LShoulder.x && LShoulder.y && LShoulder.z)
        hasLShoulder = true;
    if (RShoulder.x && RShoulder.y && RShoulder.z)
        hasRShoulder = true;
    if (LElbow.x && LElbow.y && LElbow.z)
        hasLElbow = true;
    if (RElbow.x && RElbow.y && RElbow.z)
        hasRElbow = true;
    if (LWrist.x && LWrist.y && LWrist.z)
        hasLWrist = true;
    if (RWrist.x && RWrist.y && RWrist.z)
        hasRWrist = true;
    if (LHip.x && LHip.y && LHip.z)
        hasLHip = true;
    if (RHip.x && RHip.y && RHip.z)
        hasRHip = true;

    /* adapt primitive radius parameter */
    if (hasLHip && hasRHip && hasLShoulder && hasRShoulder)
        primitive_radius_ = std::abs( distance(LHip, RHip) - distance(LShoulder, RShoulder) ) / 2;
    else
        primitive_radius_ = default_primitive_radius_;

    if (primitive_radius_ > primitive_radius_adaptation_limit_)
        primitive_radius_ = default_primitive_radius_;

    /* adapt basic limb safety radius parameter */
    if (hasLShoulder && hasLElbow && hasLWrist && hasRShoulder && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( std::max( distance(LShoulder, LElbow), distance(RShoulder, RElbow) ), std::max( distance(LElbow, LWrist), distance(RElbow, RWrist) ) );
    else if (hasLElbow && hasLWrist && hasRShoulder && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( distance(RShoulder, RElbow), std::max( distance(LElbow, LWrist), distance(RElbow, RWrist) ) );
    else if (hasLShoulder && hasLElbow && hasLWrist && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( distance(LShoulder, LElbow), std::max( distance(LElbow, LWrist), distance(RElbow, RWrist) ) );
    else if (hasLShoulder && hasLElbow && hasLWrist && hasRShoulder && hasRElbow)
        basic_limb_safety_radius_ = std::max( std::max( distance(LShoulder, LElbow), distance(RShoulder, RElbow) ), distance(LElbow, LWrist) );
    else if (hasLShoulder && hasLElbow && hasRShoulder && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( std::max( distance(LShoulder, LElbow), distance(RShoulder, RElbow) ), distance(RElbow, RWrist) );
    else if (hasLShoulder && hasLElbow && hasLWrist)
        basic_limb_safety_radius_ = std::max( distance(LShoulder, LElbow), distance(LElbow, LWrist) );
    else if (hasLShoulder && hasLElbow && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( distance(LShoulder, LElbow), distance(RElbow, RWrist) );
    else if (hasLElbow && hasLWrist && hasRShoulder && hasRElbow)
        basic_limb_safety_radius_ = std::max( distance(RShoulder, RElbow), distance(LElbow, LWrist) );
    else if (hasRShoulder && hasRElbow && hasRWrist)
        basic_limb_safety_radius_ = std::max( distance(RShoulder, RElbow), distance(RElbow, RWrist) );
    else
        basic_limb_safety_radius_ = default_basic_limb_safety_radius_;

    if (basic_limb_safety_radius_ > basic_limb_safety_radius_adaptation_limit_)
        basic_limb_safety_radius_ = default_basic_limb_safety_radius_;

    // ROS_INFO("primitive_radius: %f, basic_limb_safety_radius: %f", primitive_radius_, basic_limb_safety_radius_);
}