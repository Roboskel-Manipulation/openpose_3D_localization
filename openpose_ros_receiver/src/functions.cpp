#include "openpose_ros_receiver.hpp"

void humanListPointcloudSkeletonCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pPCL, const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg)
{
    if (list_msg && pPCL)
    {
        /* log skeletons to file */
        std::ofstream /*logfile, opfile,*/ tfedFile;
        int writen = 0;

        static tf::TransformListener tfListener;
        static tf::StampedTransform baseLinkTransform;

        try
        {
            /* take the TF subtree that we want for the transformations */
            tfListener.waitForTransform("base_link", "zed_left_camera_frame", ros::Time(0), ros::Duration(TF_WAIT));
            tfListener.lookupTransform("base_link", "zed_left_camera_frame", ros::Time(0), baseLinkTransform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        for (uint32_t i = 0; i < list_msg->num_humans; i++)
        {
            if (list_msg->human_list[i].num_body_key_points_with_non_zero_prob)
            {
                if (writen == 0)
                {
                    /* current date/time based on current system */
                    time_t now = time(0);
                    /* convert now to string form */
                    char* dt = ctime(&now);
                    std::stringstream strstream/*, strstream1, strstream2*/;
                    strstream << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/tfed " << dt << ".txt";
                    tfedFile.open(strstream.str(), std::ofstream::out);
                    // strstream1 << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/raw " << dt << ".txt";
                    // logfile.open(strstream1.str(), std::ofstream::out);
                    // strstream2 << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/OP " << dt << ".txt";
                    // opfile.open(strstream2.str(), std::ofstream::out);
                }

                // logfile << "Body " << i << " keypoints:" << std::endl;
                // opfile << "Body " << i << " keypoints:" << std::endl;
                tfedFile << "Body " << i << " keypoints:" << std::endl;

                // /* Probabilities check, do not log invalid "ghost" skeletons */
                // double bodyAvgProb, bodyTotalProb = 0.0;

                // for (uint32_t j = 0; j < 25; j++)
                //     bodyTotalProb += list_msg->human_list[i].body_key_points_with_prob[j].prob;

                // bodyAvgProb = bodyTotalProb / ((float) list_msg->human_list[i].num_body_key_points_with_non_zero_prob);

                if (1 /*bodyAvgProb > MIN_PROB_THRESHOLD*/)
                {
                    /* broadcast transform locally */
                    static tf::Transform localTransform;
                    double x = 0.0, y = 0.0, z = 0.0, prob = 0.0;

                    for (uint32_t j = 0; j < 25; j++)
                    {
                        if (!std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].z))
                        {
                            pcl::PointXYZ p1 = pPCL->at(std::roundl(list_msg->human_list[i].body_key_points_with_prob[j].x), std::roundl(list_msg->human_list[i].body_key_points_with_prob[j].y));
                            x = p1.x; y = p1.y; z = p1.z;

                            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                                continue;

                            try
                            {
                                /* locally */
                                localTransform.setOrigin( tf::Vector3(x, y, z) );
                                tf::Quaternion localQuat(0, 0, 0, 1);
                                localTransform.setRotation(localQuat);
                                tfListener.setTransform( tf::StampedTransform(localTransform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(j)) );

                                /* LOOKUP LOCAL TRANSFORM */
                                // std::vector<std::string> strVec;
                                // tfListener.getFrameStrings(strVec);
                                // for (std::string s : strVec)
                                //     ROS_INFO("%s", s.c_str());
                                tfListener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(TF_WAIT));
                                tfListener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), baseLinkTransform);

                                /* log */
                                // logfile << "raw kp " << getPoseBodyPartMappingBody25(j) << ": x=" << x << " y=" << y << " z=" << z << " prob=" << list_msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;
                                // opfile << "OP kp " << getPoseBodyPartMappingBody25(j) << ": x=" << list_msg->human_list[i].body_key_points_with_prob[j].x << " y=" << list_msg->human_list[i].body_key_points_with_prob[j].y << " z=" << list_msg->human_list[i].body_key_points_with_prob[j].z << " prob=" << list_msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;     
                                if (!std::isnan(baseLinkTransform.getOrigin().x()) && !std::isnan(baseLinkTransform.getOrigin().y()) && !std::isnan(baseLinkTransform.getOrigin().z()))
                                {
                                    /* process transform's timestamp */
                                    ros::Time now = ros::Time::now(), transformStamp = baseLinkTransform.stamp_;
                                    ros::Duration timeDiff = now - transformStamp;
                                    /* check if we are dealing with a frame old enough (e.g. > 2 sec) to be considered unreliable */
                                    if (timeDiff > ros::Duration(RELIABILITY_THRESHOLD))
                                    {
                                        ROS_WARN("Transform older than %f seconds detected", RELIABILITY_THRESHOLD);
                                        continue;
                                    }
                                    tfedFile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << baseLinkTransform.getOrigin().x() << " y=" << baseLinkTransform.getOrigin().y()
                                            << " z=" << baseLinkTransform.getOrigin().z() << std::endl;
                                }
                            }
                            catch (tf::TransformException &ex)
                            {
                                ROS_ERROR("%s", ex.what());
                                continue;
                            }
                            // ROS_INFO("1");
                        }
                    }
                }

                // logfile << std::endl << std::endl;
                // opfile << std::endl << std::endl;
                tfedFile << std::endl << std::endl;

                writen++;
            }
        }
        
        // logfile.close();
        // opfile.close();
        tfedFile.close();
    }
}

void listenForSkeleton(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    // ROS_WARN("listenForSkeleton IN");
    // /* log skeletons to file */
    // std::ofstream logfile;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    int retry = 0/*, writen = 0*/;
    // ROS_WARN("PERSON keypoints:");
    for (uint32_t j = 0; j < 25; j++)
    {
        try
        {
            /* LISTEN FOR TRANSFORMATIONS */
            listener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(TF_WAIT));
            listener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), transform);

            /* process transform's timestamp */
            if (!std::isnan(transform.getOrigin().x()) && !std::isnan(transform.getOrigin().y()) && !std::isnan(transform.getOrigin().z()))
            {

                ros::Time now = ros::Time::now(), transformStamp = transform.stamp_;
                ros::Duration timeDiff = now - transformStamp;
                /* check if we are dealing with a frame old enough (e.g. > 2 sec) to be considered unreliable */
                if (timeDiff > ros::Duration(RELIABILITY_THRESHOLD))
                {
                    ROS_WARN("Transform older than %f seconds detected", RELIABILITY_THRESHOLD);
                    continue;
                }
            }
        }
        catch (tf::TransformException &ex)
        {
            /* if listening for transformation fails do not instantly give up on it, give it some more attempts */
            if (retry < MAX_RETRY)
            {
                j--;
                retry++;
            }
            else
            {
                retry = 0;
                // ROS_ERROR("%s",ex.what());
            }
            continue;
        }

        // if (writen == 0)
        // {
        //     /* current date/time based on current system */
        //     time_t now = time(0);
        //     /* convert now to string form */
        //     char* dt = ctime(&now);
        //     std::stringstream strstream;
        //     strstream << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/tfed " << dt << ".txt";
        //     logfile.open(strstream.str(), std::ofstream::out);
        //     logfile << "Body keypoints:" << std::endl;
        // }

        // if (!std::isnan(transform.getOrigin().x()) && !std::isnan(transform.getOrigin().y()) && !std::isnan(transform.getOrigin().z()))
        // {
        //     logfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y()
        //             << " z=" <<  transform.getOrigin().z() << std::endl;
        // }

        // writen++;
    }
    // ROS_WARN("---");

    // logfile.close();
    // ROS_WARN("listenForSkeleton OUT");
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}