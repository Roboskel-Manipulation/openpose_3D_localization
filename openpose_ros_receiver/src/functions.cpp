#include "openpose_ros_receiver.hpp"

/* Global Variables */
bool tfSubtree, logging;
int totalNans, callbackVisits;

void humanListPointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pPCL, const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg, const ros::Publisher& robotFrameCoordsPub)
{
    if (list_msg && pPCL)
    {
        if (logging)
        {
            /* log skeletons to file */
            std::ofstream tfedFile;
            int writen = 0;
            std::stringstream strstream;

            static tf::TransformListener tfListener;
            static tf::StampedTransform baseLinkTransform;

            if (!tfSubtree)
            {
                try
                {
                    /* take the TF subtree that we want for the transformations */
                    tfListener.waitForTransform("base_link", "zed_left_camera_frame", ros::Time(0), ros::Duration(TF_WAIT));
                    tfListener.lookupTransform("base_link", "zed_left_camera_frame", ros::Time(0), baseLinkTransform);

                    tfSubtree = true;
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
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
                        strstream << "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/tfed " << dt << ".txt";
                        tfedFile.open(strstream.str(), std::ofstream::out);
                    }

                    tfedFile << "Body " << i << " keypoints:" << std::endl;

                    /* broadcast transform locally */
                    static tf::Transform localTransform;
                    double x = 0.0, y = 0.0, z = 0.0, prob = 0.0, x_pix = 0.0, y_pix = 0.0, z0 = 0.0;

                    for (uint32_t j = 0; j < 25; j++)
                    {
                        x_pix = list_msg->human_list[i].body_key_points_with_prob[j].x; y_pix = list_msg->human_list[i].body_key_points_with_prob[j].y;

                        if (!std::isnan(x_pix) && !std::isnan(y_pix))
                        {
                            /* Get an average of neighboring points coordinates for more precise x, y, z */
                            // x --> width, y --> height
                            int divisors = 0;
                            pcl::PointXYZ p;

                            /* our point */
                            p = pPCL->at(x_pix, y_pix);
                            x = p.x; y = p.y; z = p.z;
                            z0 = z;
                            divisors++;
                            /* P: our point, *: one of our point's neighbors
                                * * *
                                    ***
                                **P**
                                    ***
                                * * *
                            */
                        //    int localNans = 0;
                            /* our point's 1st class neighbors */
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-1, y_pix) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-1, y_pix) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-1, y_pix) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix, y_pix-1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix, y_pix-1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix, y_pix-1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-1, y_pix-1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-1, y_pix-1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-1, y_pix-1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+1, y_pix) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+1, y_pix) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+1, y_pix) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix, y_pix+1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix, y_pix+1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix, y_pix+1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+1, y_pix+1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+1, y_pix+1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+1, y_pix+1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-1, y_pix+1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-1, y_pix+1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-1, y_pix+1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+1, y_pix-1) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+1, y_pix-1) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+1, y_pix-1) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            /* some of our point's 2nd class neighbors */
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-2, y_pix) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-2, y_pix) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-2, y_pix) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix, y_pix-2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix, y_pix-2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix, y_pix-2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-2, y_pix-2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-2, y_pix-2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-2, y_pix-2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+2, y_pix) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+2, y_pix) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+2, y_pix) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix, y_pix+2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix, y_pix+2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix, y_pix+2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+2, y_pix+2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+2, y_pix+2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+2, y_pix+2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix-2, y_pix+2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix-2, y_pix+2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix-2, y_pix+2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                                // else
                                // {
                                //     ROS_WARN("***");
                                //     if (std::isnan(p.x)) ROS_WARN("%s: at(x_pix+2, y_pix-2) x is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.y)) ROS_WARN("%s: at(x_pix+2, y_pix-2) y is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     if (std::isnan(p.z)) ROS_WARN("%s: at(x_pix+2, y_pix-2) z is nan", getPoseBodyPartMappingBody25(j).c_str());
                                //     localNans++; totalNans++;
                                // }
                            }
                            // ROS_INFO("----------------** %d local nan values **----------------", localNans);

                            if (divisors)
                            {
                                z /= (double) divisors;
                                /* for edge cases */
                                if (z > UPPER_VARIATION_THRESH * z0 || z < LOWER_VARIATION_THRESH * z0) z = z0;
                            }

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
                                tfListener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(TF_WAIT));
                                tfListener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), baseLinkTransform);

                                /* log */
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
                                    strstream << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << baseLinkTransform.getOrigin().x() << " y=" << baseLinkTransform.getOrigin().y()
                                            << " z=" << baseLinkTransform.getOrigin().z() << std::endl;
                                }
                            }
                            catch (tf::TransformException &ex)
                            {
                                ROS_ERROR("%s", ex.what());
                                continue;
                            }
                        }
                    }

                    tfedFile << std::endl << std::endl;

                    writen++;
                }
            }
            
            tfedFile.close();

            /* for profiling */
            // ROS_INFO("%s", strstream.str().c_str());
            std_msgs::String stringMsg;
            stringMsg.data = strstream.str();
            robotFrameCoordsPub.publish(stringMsg);
            // ROS_WARN("----------------** %d total nan values in %d callback visits **----------------", totalNans, callbackVisits);
        }
        else
        {
            std::stringstream strstream;

            static tf::TransformListener tfListener;
            static tf::StampedTransform baseLinkTransform;

            if (!tfSubtree)
            {
                try
                {
                    /* take the TF subtree that we want for the transformations */
                    tfListener.waitForTransform("base_link", "zed_left_camera_frame", ros::Time(0), ros::Duration(TF_WAIT));
                    tfListener.lookupTransform("base_link", "zed_left_camera_frame", ros::Time(0), baseLinkTransform);

                    tfSubtree = true;
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
            }

            for (uint32_t i = 0; i < list_msg->num_humans; i++)
            {
                if (list_msg->human_list[i].num_body_key_points_with_non_zero_prob)
                {
                    /* broadcast transform locally */
                    static tf::Transform localTransform;
                    double x = 0.0, y = 0.0, z = 0.0, prob = 0.0, x_pix = 0.0, y_pix = 0.0, z0 = 0.0;

                    for (uint32_t j = 0; j < 25; j++)
                    {
                        x_pix = list_msg->human_list[i].body_key_points_with_prob[j].x; y_pix = list_msg->human_list[i].body_key_points_with_prob[j].y;

                        if (!std::isnan(x_pix) && !std::isnan(y_pix))
                        {
                            /* Get an average of neighboring points coordinates for more precise x, y, z */
                            // x --> width, y --> height
                            int divisors = 0;
                            pcl::PointXYZ p;

                            /* our point */
                            p = pPCL->at(x_pix, y_pix);
                            x = p.x; y = p.y; z = p.z;
                            z0 = z;
                            divisors++;
                            /* P: our point, *: one of our point's neighbors
                                * * *
                                    ***
                                **P**
                                    ***
                                * * *
                            */
                            /* our point's 1st class neighbors */
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix-1 >= 0 && x_pix-1 < IMG_PIXEL_WIDTH) && (y_pix+1 >= 0 && y_pix+1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1, y_pix+1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+1 >= 0 && x_pix+1 < IMG_PIXEL_WIDTH) && (y_pix-1 >= 0 && y_pix-1 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1, y_pix-1);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            /* some of our point's 2nd class neighbors */
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix-2 >= 0 && x_pix-2 < IMG_PIXEL_WIDTH) && (y_pix+2 >= 0 && y_pix+2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2, y_pix+2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }
                            if ((x_pix+2 >= 0 && x_pix+2 < IMG_PIXEL_WIDTH) && (y_pix-2 >= 0 && y_pix-2 < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2, y_pix-2);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    z += p.z;
                                    divisors++;
                                }
                            }

                            if (divisors)
                            {
                                z /= (double) divisors;
                                /* for edge cases */
                                if (z > UPPER_VARIATION_THRESH * z0 || z < LOWER_VARIATION_THRESH * z0) z = z0;
                            }

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
                                tfListener.waitForTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), ros::Duration(TF_WAIT));
                                tfListener.lookupTransform("base_link", getPoseBodyPartMappingBody25(j), ros::Time(0), baseLinkTransform);

                                /* log */
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
                                    strstream << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << baseLinkTransform.getOrigin().x() << " y=" << baseLinkTransform.getOrigin().y()
                                            << " z=" << baseLinkTransform.getOrigin().z() << std::endl;
                                }
                            }
                            catch (tf::TransformException &ex)
                            {
                                ROS_ERROR("%s", ex.what());
                                continue;
                            }
                        }
                    }
                }
            }
            
            /* for profiling */
            // ROS_INFO("%s", strstream.str().c_str());
            std_msgs::String stringMsg;
            stringMsg.data = strstream.str();
            robotFrameCoordsPub.publish(stringMsg);
        }
    
        callbackVisits++;
    }
}

void listenForSkeleton(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    int retry = 0;
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
            }
            continue;
        }
    }
}

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}
