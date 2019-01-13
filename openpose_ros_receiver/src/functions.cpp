#include "openpose_ros_receiver.hpp"

/* Global variables */
bool pointcloud_flag;
std::vector<double> xVec, yVec, zVec;
pcl::PointCloud<pcl::PointXYZ> pclCp;

void initGlobalVars()
{
    pointcloud_flag = false;
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
}

void reInitGlobalVars()
{
    xVec = std::vector<double>(0);
    yVec = std::vector<double>(0);
    zVec = std::vector<double>(0);
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

        for (uint32_t i = 0; i < list_msg->num_humans; i++)
        {
            if (list_msg->human_list[i].num_body_key_points_with_non_zero_prob)
            {
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
                        }
                    }

                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    int j, ext_i = i;
                    double x, y, z;

                    for (uint32_t i = 0; i < xVec.size(); i++)
                    {
                        if (!xVec.at(i) && !yVec.at(i) && !zVec.at(i))
                        {
                            x = 0.0; y = 0.0; z = 0.0;
                        }
                        else
                        {
                            pcl::PointXYZ p1 = pPCL->at(xVec.at(i), yVec.at(i));
                            x = p1.x; y = p1.y; z = p1.z;

                            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                                continue;

                            /* BROADCAST TRANSFORMATIONS */
                            transform.setOrigin( tf::Vector3(x, y, z) );
                            tf::Quaternion q(0, 0, 0, 1);
                            transform.setRotation(q);
                            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(i)) );

                            /* report */
                            ROS_INFO("SEND time=%d, from=%s, to=%s, x=%f, y=%f, z=%f",
                                    ros::Time::now().toSec(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(i), transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

                            /* log */
                            temp_list_msg.human_list[ext_i].body_key_points_with_prob[i].x = x;
                            temp_list_msg.human_list[ext_i].body_key_points_with_prob[i].y = y;
                            temp_list_msg.human_list[ext_i].body_key_points_with_prob[i].z = z;
                        }
                    }

                    /* Re-Initialize Global variables */
                    reInitGlobalVars();
                }
            }

            writeSkeletonToFile(temp_list_msg);
        }
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
                else
                {
                    /* report */
                    if (!std::isnan(transform.getOrigin().x()) && !std::isnan(transform.getOrigin().y()) && !std::isnan(transform.getOrigin().z()))
                        ROS_INFO("RCV: stamp=%d, from=%s, to=%s, x=%f, y=%f, z=%f",
                               transform.stamp_.toSec(), transform.frame_id_, transform.child_frame_id_, transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
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
                ROS_ERROR("%s",ex.what());
                // ros::Duration(0.1).sleep();
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

    outfile.close();
    logfile.close();
    // ROS_WARN("listenForSkeleton OUT");
}

void writeSkeletonToFile(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
    /* log skeletons to file */
    std::ofstream outfile, logfile;

    int writen = 0;
    for (uint32_t i = 0; i < msg.num_humans; i++)
    {
        if (msg.human_list[i].num_body_key_points_with_non_zero_prob)
        {
            if (writen == 0)
            {
                outfile.open("/home/gkamaras/openpose_ros_receiver_raw.txt", std::ofstream::out);
                /* current date/time based on current system */
                time_t now = time(0);
                /* convert now to string form */
                char* dt = ctime(&now);
                std::stringstream strstream;
                strstream << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/raw " << dt << ".txt";
                logfile.open(strstream.str(), std::ofstream::out);
            }

            outfile << "Body " << i << " keypoints:" << std::endl;
            logfile << "Body " << i << " keypoints:" << std::endl;

            /* Probabilities check, do not log invalid "ghost" skeletons */
            double bodyAvgProb, bodyTotalProb = 0.0;
            int i = 0;  // for development

            for (uint32_t j = 0; j < msg.human_list[i].num_body_key_points_with_non_zero_prob; j++)
                bodyTotalProb += msg.human_list[i].body_key_points_with_prob[j].prob;

            bodyAvgProb = bodyTotalProb / ((float) msg.human_list[i].num_body_key_points_with_non_zero_prob);

            if (bodyAvgProb > MIN_PROB_THRESHOLD)
            {
                i++;    // for development phase

                for (uint32_t j = 0; j < 25; j++)
                {
                    if (!std::isnan(msg.human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg.human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg.human_list[i].body_key_points_with_prob[j].z))
                    {
                        outfile << "raw kp " << getPoseBodyPartMappingBody25(j) << ": x=" << msg.human_list[i].body_key_points_with_prob[j].x << " y=" <<  msg.human_list[i].body_key_points_with_prob[j].y
                                << " z=" <<  msg.human_list[i].body_key_points_with_prob[j].z << " prob=" <<  msg.human_list[i].body_key_points_with_prob[j].prob << std::endl;
                        logfile << "raw kp " << getPoseBodyPartMappingBody25(j) << ": x=" << msg.human_list[i].body_key_points_with_prob[j].x << " y=" <<  msg.human_list[i].body_key_points_with_prob[j].y
                                << " z=" <<  msg.human_list[i].body_key_points_with_prob[j].z << " prob=" <<  msg.human_list[i].body_key_points_with_prob[j].prob << std::endl;
                    }
                } 
            }

            assert(i < 5);  // for development phase

            outfile << std::endl << std::endl;
            logfile << std::endl << std::endl;

            writen++;
        }
    }

    outfile.close();
    logfile.close();
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}