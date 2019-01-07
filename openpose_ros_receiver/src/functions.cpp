#include "openpose_ros_receiver.hpp"

/* Global variables */
bool /*new_data_flag,*/ broadcast_flag, pointcloud_flag;
std::vector<double> xVec, yVec, zVec;
sensor_msgs::PointCloud2 pcl;
uint32_t ind;

void initGlobalVars()
{
    // new_data_flag = false;
    broadcast_flag = false;
    pointcloud_flag = false;
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
    ind = 0;
}

void reInitGlobalVars()
{
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
}

void humanListPointcloudSkeletonCallback(const sensor_msgs::PointCloud2::ConstPtr& pPCL2, const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg, const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    if (pPCL2)
    {
        ROS_WARN("humanListBroadcastCallback IN");

        pcl = *pPCL2;

        pointcloud_flag = true;

        ROS_WARN("humanListBroadcastCallback OUT");
    }

    if (list_msg && pointcloud_flag)
    {
        ROS_WARN("pointcloudCallback IN");

        for (uint32_t i = 0; i < list_msg->num_humans; i++)
        {
            if (list_msg->human_list[i].num_body_key_points_with_non_zero_prob)
            {
                /* Probabilities check, do not log invalid "ghost" skeletons */
                double avg_prob, total_prob = 0.0;

                for (uint32_t j = 0; j < 25; j++)
                    total_prob += list_msg->human_list[i].body_key_points_with_prob[j].prob;

                avg_prob = total_prob / list_msg->human_list[i].num_body_key_points_with_non_zero_prob;

                if (avg_prob > 0.0 /*MIN_PROB_THRESHOLD*/)
                {
                    for (uint32_t j = 0; j < 25; j++)
                    {
                        if (!std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].z))
                        {
                            // ROS_INFO("Filling human %d member %d", i, j);
                            xVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].x);
                            yVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].y);
                            zVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].z);
                        }
                    }

                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    int j;

                    for (uint32_t i = 0; i < xVec.size(); i++)
                    {
                        j = xVec.at(i) + (yVec.at(i)) * pPCL2->width;
                        // ROS_INFO("i = %d, j = %d", i, j);
                        double x = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[0].offset];
                        double y = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[1].offset];
                        double z = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[2].offset];
                        // ROS_INFO("x = %f, y = %f, z = %f", x, y, z);

                        /* BROADCAST TRANSFORMATIONS */
                        transform.setOrigin( tf::Vector3(x, y, z) );
                        tf::Quaternion q(0, 0, 0, 1);
                        transform.setRotation(q);
                        br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(i)) );
                        // ROS_INFO("Just send transform for %s frame", getPoseBodyPartMappingBody25(i).c_str());
                    }

                    if (xVec.size())
                        broadcast_flag = true;

                    /* Re-Initialize Global variables */
                    reInitGlobalVars();
                }
            }
        }

        ROS_WARN("pointcloudCallback OUT");
    }

    // if (msg && broadcast_flag)
    // {
    //     ROS_WARN("listenForSkeleton IN");

    //     /* log skeletons to file */
    //     std::ofstream outfile;
    //     outfile.open("/home/gkamaras/openpose_ros_receiver.txt", std::ofstream::trunc);

    //     tf::TransformListener listener;
    //     tf::StampedTransform transform;
    //     // ros::Rate rate(1000.0);

    //     int retry = 0;
    //     outfile << "Body keypoints:" << std::endl;
    //     for (uint32_t j = 0; j < 25; j++)
    //     {
    //         // rate.sleep();
    //         try
    //         {
    //             /* LISTEN FOR TRANSFORMATIONS */
    //             listener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(0.1));
    //             listener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), transform);
    //         }
    //         catch (tf::TransformException &ex)
    //         {
    //             ROS_ERROR("%s",ex.what());
    //             // ros::Duration(0.1).sleep();
    //             continue;
    //         }

    //         outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y() << " z=" <<  transform.getOrigin().z() << std::endl;
    //     }

    //     outfile.close();

    //     broadcast_flag = false;

    //     ROS_WARN("listenForSkeleton OUT");
    // }
}

// void humanListBroadcastCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
// {
//     ROS_WARN("humanListBroadcastCallback IN");
//     // static tf::TransformBroadcaster br;
//     // tf::Transform transform;
//     // ros::Rate rate(5.0);

//     for (uint32_t i = 0; i < msg->num_humans; i++)
//     {
//         if (msg->human_list[i].num_body_key_points_with_non_zero_prob)
//         {
//             /* Probabilities check, do not log invalid "ghost" skeletons */
//             double avg_prob, total_prob = 0.0;

//             for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
//                 total_prob += msg->human_list[i].body_key_points_with_prob[j].prob;

//             avg_prob = total_prob / msg->human_list[i].num_body_key_points_with_non_zero_prob;

//             if (avg_prob > 0.0 /*MIN_PROB_THRESHOLD*/)
//             {
//                 for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
//                 {
//                     // rate.sleep();
//                     if (!std::isnan(msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].z))
//                     {
//                         ROS_WARN("Filling");
//                         // /* BROADCAST TRANSFORMATIONS */
//                         // transform.setOrigin( tf::Vector3(msg->human_list[i].body_key_points_with_prob[j].x, msg->human_list[i].body_key_points_with_prob[j].y, msg->human_list[i].body_key_points_with_prob[j].z) );
//                         // tf::Quaternion q(0, 0, 0, 1);
//                         // transform.setRotation(q);
//                         // br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(j)/*+"_"+std::to_string(i)*/) );
//                         xVec.push_back(msg->human_list[i].body_key_points_with_prob[j].x);
//                         yVec.push_back(msg->human_list[i].body_key_points_with_prob[j].y);
//                         zVec.push_back(msg->human_list[i].body_key_points_with_prob[j].z);
//                     }
//                 }
//             }

//             new_data_flag = true;
//             while (new_data_flag) {;}

//             /* Re-Initialize Global variables */
//             reInitGlobalVars();
//         }
//     }

//     // writeSkeletonToFile(msg);
//     ROS_WARN("humanListBroadcastCallback OUT");
// }

// void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pPCL2)
// {
//     ROS_WARN("pointcloudCallback IN");
//     while (!new_data_flag) {;}
//     if (new_data_flag)
//     {
//         static tf::TransformBroadcaster br;
//         tf::Transform transform;
//         int j;

//         for (uint32_t i = 0; i < xVec.size(); i++)
//         {
//             j = xVec.at(i) + (yVec.at(i) - 1) * pPCL2->width;

//             double x = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[0].offset];
//             double y = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[1].offset];
//             double z = pPCL2->data[j * pPCL2->point_step + pPCL2->fields[2].offset];

//             /* BROADCAST TRANSFORMATIONS */
//             transform.setOrigin( tf::Vector3(x, y, z) );
//             tf::Quaternion q(0, 0, 0, 1);
//             transform.setRotation(q);
//             br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(j)) );
//         }
        
//         new_data_flag = false;
//     }

//     ROS_WARN("pointcloudCallback OUT");
// }

void listenForSkeleton(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    ROS_WARN("listenForSkeleton IN");
    /* log skeletons to file */
    std::ofstream outfile;
    outfile.open("/home/gkamaras/openpose_ros_receiver.txt", std::ofstream::trunc);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(1000.0);


    int retry = 0;
    outfile << "Body keypoints:" << std::endl;
    for (uint32_t j = 0; j < 25; j++)
    {
        rate.sleep();
        try
        {
            /* LISTEN FOR TRANSFORMATIONS */
            listener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(0.1));
            listener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            // /* if listening for transformation fails do not instantly give up on it, give it some more attempts */
            // if (retry < MAX_RETRY)
            // {
            //     j--;
            //     retry++;
            // }
            // else
            //     retry = 0;
            ros::Duration(0.1).sleep();
            continue;
        }

        outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y() << " z=" <<  transform.getOrigin().z() << std::endl;
    }

    outfile.close();

    ROS_WARN("listenForSkeleton OUT");
}

void writeSkeletonToFile(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    /* log skeletons to file */
    std::ofstream outfile;
    outfile.open("/home/gkamaras/openpose_ros_receiver.txt", std::ofstream::trunc);

    for (uint32_t i = 0; i < msg->num_humans; i++)
    {
        if (msg->human_list[i].num_body_key_points_with_non_zero_prob)
        {
            tf::TransformListener listener;
            tf::StampedTransform transform;
            ros::Rate rate(5.0);

            /* Probabilities check, do not log invalid "ghost" skeletons */
            double avg_prob, total_prob = 0.0;
            int i = 0;  // for development

            for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
                total_prob += msg->human_list[i].body_key_points_with_prob[j].prob;

            avg_prob = total_prob / msg->human_list[i].num_body_key_points_with_non_zero_prob;

            if (avg_prob > 0.0 /*MIN_PROB_THRESHOLD*/)
            {
                i++;    // for development phase

                int retry = 0;
                outfile << "Body " << i << " keypoints:" << std::endl;
                for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
                {
                    if (!std::isnan(msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].z))
                    {
                        rate.sleep();
                        try
                        {
                            /* LISTEN FOR TRANSFORMATIONS */
                            listener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j)/*+"_"+std::to_string(i)*/, ros::Time(0), ros::Duration(1.0));
                            listener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j)/*+"_"+std::to_string(i)*/, ros::Time(0), transform);
                        }
                        catch (tf::TransformException &ex)
                        {
                            ROS_ERROR("%s",ex.what());
                            // /* if listening for transformation fails do not instantly give up on it, give it some more attempts */
                            // if (retry < MAX_RETRY)
                            // {
                            //     j--;
                            //     retry++;
                            // }
                            // else
                            //     retry = 0;
                            ros::Duration(1.0).sleep();
                            continue;
                        }

                        outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y()
                                << " z=" <<  transform.getOrigin().y() << " prob=" <<  msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;
                    }
                }
            }

            assert(i < 5);  // for development phase
        }

        if (msg->human_list[i].num_face_key_points_with_non_zero_prob)
        {
            outfile << "Face keypoints:" << std::endl;
            for (uint32_t j = 0; j < msg->human_list[i].num_face_key_points_with_non_zero_prob; j++)
            {
                if (!std::isnan(msg->human_list[i].face_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].face_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].face_key_points_with_prob[j].z))
                {
                    outfile << "kp " << j << ": x=" <<  msg->human_list[i].face_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].face_key_points_with_prob[j].y
                            << " z=" <<  msg->human_list[i].face_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].face_key_points_with_prob[j].prob << std::endl;
                }
            }
        }

        if (msg->human_list[i].num_right_hand_key_points_with_non_zero_prob)
        {
            outfile << "Right hand keypoints:" << std::endl;
            for (uint32_t j = 0; j < msg->human_list[i].num_right_hand_key_points_with_non_zero_prob; j++)
            {
                if (!std::isnan(msg->human_list[i].right_hand_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].right_hand_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].right_hand_key_points_with_prob[j].z))
                {
                    outfile << "kp " << j << ": x=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].y
                            << " z=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].prob << std::endl;
                }
            }
        }

        if (msg->human_list[i].num_left_hand_key_points_with_non_zero_prob)
        {
            outfile << "Left hand keypoints:" << std::endl;
            for (uint32_t j = 0; j < msg->human_list[i].num_left_hand_key_points_with_non_zero_prob; j++)
            {
                if (!std::isnan(msg->human_list[i].left_hand_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].left_hand_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].left_hand_key_points_with_prob[j].z))
                {
                    outfile << "kp " << j << ": x=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].y
                            << " z=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].prob << std::endl;
                }
            }
        }

        outfile << std::endl << std::endl;
    }

    outfile.close();
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}