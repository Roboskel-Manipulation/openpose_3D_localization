#include "openpose_ros_receiver.hpp"

/* Global variables */
bool pointcloud_flag;
std::vector<double> xVec, yVec, zVec, probVec;
pcl::PointCloud<pcl::PointXYZ> pclCp;

void initGlobalVars()
{
    pointcloud_flag = false;
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
    probVec = std::vector<double>(0);
}

void reInitGlobalVars()
{
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
    probVec = std::vector<double>(0);
}

void humanListPointcloudSkeletonCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pPCL, const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg)
{
    if (pPCL)
    {
        // ROS_WARN("pointcloudCallback IN");
        pclCp = *pPCL;
        pointcloud_flag = true;
        // ROS_WARN("pointcloudCallback OUT");
    }

    if (list_msg && pointcloud_flag)
    {
        // ROS_WARN("humanListBroadcastCallback IN");
        openpose_ros_msgs::OpenPoseHumanList temp_list_msg = *list_msg;

        /* log skeletons to file */
        std::ofstream outfile, logfile, opfile;
        int writen = 0;

        for (uint32_t i = 0; i < list_msg->num_humans; i++)
        {
            if (list_msg->human_list[i].num_body_key_points_with_non_zero_prob)
            {
                if (writen == 0)
                {
                    outfile.open("/home/gkamaras/openpose_ros_receiver_raw.txt", std::ofstream::out);
                    /* current date/time based on current system */
                    time_t now = time(0);
                    /* convert now to string form */
                    char* dt = ctime(&now);
                    std::stringstream strstream1, strstream2;
                    strstream1 << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/raw " << dt << ".txt";
                    logfile.open(strstream1.str(), std::ofstream::out);
                    strstream2 << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/OP " << dt << ".txt";
                    opfile.open(strstream2.str(), std::ofstream::out);
                }

                outfile << "Body " << i << " keypoints:" << std::endl;
                logfile << "Body " << i << " keypoints:" << std::endl;
                opfile << "Body " << i << " keypoints:" << std::endl;

                /* Probabilities check, do not log invalid "ghost" skeletons */
                double bodyAvgProb, bodyTotalProb = 0.0;

                for (uint32_t j = 0; j < 25; j++)
                    bodyTotalProb += list_msg->human_list[i].body_key_points_with_prob[j].prob;

                bodyAvgProb = bodyTotalProb / ((float) list_msg->human_list[i].num_body_key_points_with_non_zero_prob);

                if (bodyAvgProb > MIN_PROB_THRESHOLD)
                {
                    for (uint32_t j = 0; j < 25; j++)
                    {
                        if (!std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(list_msg->human_list[i].body_key_points_with_prob[j].z))
                        {
                            // ROS_INFO("Filling human %d member %d", i, j);
                            xVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].x);
                            yVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].y);
                            zVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].z);
                            probVec.push_back(list_msg->human_list[i].body_key_points_with_prob[j].prob);
                        }
                        else
                        {
                            xVec.push_back(0.0);
                            yVec.push_back(0.0);
                            zVec.push_back(0.0);
                            probVec.push_back(0.0);
                        }
                    }

                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    double x, y, z, prob;

                    /* for debugging */
                    assert(xVec.size() == 25);

                    for (uint32_t k = 0; k < xVec.size(); k++)
                    {
                        if (!xVec.at(k) && !yVec.at(k) && !zVec.at(k))
                        {
                            x = 0.0; y = 0.0; z = 0.0; prob = 0.0;
                        }
                        else
                        {
                            pcl::PointXYZ p1 = pPCL->at(xVec.at(k), yVec.at(k));
                            x = p1.x; y = p1.y; z = p1.z; prob = probVec.at(k);

                            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                                continue;

                            /* BROADCAST TRANSFORMATIONS */
                            transform.setOrigin( tf::Vector3(x, y, z) );
                            tf::Quaternion q(0, 0, 0, 1);
                            transform.setRotation(q);
                            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(k)) );

                            /* report */
                            ROS_INFO("SEND time=%f, from=%s, to=%s, x=%f, y=%f, z=%f",
                                    ros::Time::now().toSec(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(k).c_str(), transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

                            /* log */
                            outfile << "raw kp " << getPoseBodyPartMappingBody25(k) << ": x=" << x << " y=" <<  y << " z=" <<  z << " prob=" <<  prob << std::endl;
                            logfile << "raw kp " << getPoseBodyPartMappingBody25(k) << ": x=" << x << " y=" <<  y << " z=" <<  z << " prob=" <<  prob << std::endl;
                            opfile << "OP kp " << getPoseBodyPartMappingBody25(k) << ": x=" << xVec.at(k) << " y=" << yVec.at(k) << " z=" << zVec.at(k) << " prob=" << probVec.at(k) << std::endl;      
                        }
                    }

                    /* Re-Initialize Global variables */
                    reInitGlobalVars();
                }

                outfile << std::endl << std::endl;
                logfile << std::endl << std::endl;
                opfile << std::endl << std::endl;

                writen++;
            }
        }
        
        outfile.close();
        logfile.close();
        opfile.close();
        // ROS_WARN("humanListBroadcastCallback OUT");
    }
}

void listenForSkeleton(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    // ROS_WARN("listenForSkeleton IN");
    /* log skeletons to file */
    std::ofstream outfile, logfile;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    int retry = 0, writen = 0;
    // ROS_WARN("PERSON keypoints:");
    for (uint32_t j = 0; j < 25; j++)
    {
        try
        {
            /* LISTEN FOR TRANSFORMATIONS */
            listener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(0.1));
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
                // else
                // {
                //     /* report */
                //     if (!std::isnan(transform.getOrigin().x()) && !std::isnan(transform.getOrigin().y()) && !std::isnan(transform.getOrigin().z()))
                //         ROS_INFO("RCV: stamp=%f, from=%s, to=%s, x=%f, y=%f, z=%f",
                //                transform.stamp_.toSec(), transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                // }
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
                ROS_ERROR("%s",ex.what());
            }
            continue;
        }

        if (writen == 0)
        {
            outfile.open("/home/gkamaras/openpose_ros_receiver_tfed.txt", std::ofstream::out);
            outfile << "Body keypoints:" << std::endl;
            /* current date/time based on current system */
            time_t now = time(0);
            /* convert now to string form */
            char* dt = ctime(&now);
            std::stringstream strstream;
            strstream << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/tfed " << dt << ".txt";
            logfile.open(strstream.str(), std::ofstream::out);
            logfile << "Body keypoints:" << std::endl;
        }

        if (!std::isnan(transform.getOrigin().x()) && !std::isnan(transform.getOrigin().y()) && !std::isnan(transform.getOrigin().z()))
        {
            outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y()
                    << " z=" <<  transform.getOrigin().z() << std::endl;
            logfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y()
                    << " z=" <<  transform.getOrigin().z() << std::endl;
        }

        writen++;
    }
    // ROS_WARN("---");

    outfile.close();
    logfile.close();
    // ROS_WARN("listenForSkeleton OUT");
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}