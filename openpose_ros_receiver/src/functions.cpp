#include "openpose_ros_receiver.hpp"

/* Global Variables */
bool tfSubtree, logging, pclMsg, humanListMsg;
std::string image_frame;
ros::Publisher robotFrameCoordsPub, humanReceiverPub, pointcloudDebugPub;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPCL;

void pointCloudTopicCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_msg)
{
    if (humanListMsg)
    {
        pclMsg = true;
        pPCL = pcl_msg;
        humanListMsg = false;
    }
    else
    {
        ros::spinOnce();
    }
}

void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg)
{
    if (pclMsg)
    {
        humanListMsg = true;

        std::stringstream strstream;
        int neighborhoodFactor = 1;

        openpose_ros_receiver_msgs::OpenPoseReceiverHuman humanMsg;
        if (list_msg->num_humans)
            humanMsg.num_body_key_points_with_non_zero_prob = list_msg->human_list[0].num_body_key_points_with_non_zero_prob;
        
        if (logging)
        {
            /* log skeletons to file */
            std::ofstream tfedFile;
            int writen = 0;

            static tf::TransformListener tfListener;
            static tf::StampedTransform baseLinkTransform;

            if (!tfSubtree)
            {
                try
                {
                    /* take the TF subtree that we want for the transformations */
                    tfListener.waitForTransform("base_link", image_frame, ros::Time(0), ros::Duration(TF_WAIT));
                    tfListener.lookupTransform("base_link", image_frame, ros::Time(0), baseLinkTransform);

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

                    for (uint32_t j = 0; j < 25; j++)
                    {
                        double x = 0.0, y = 0.0, z = 0.0, prob = 0.0, x_pix = 0.0, y_pix = 0.0, z_pix = 0.0, x0 = 0.0;

                        x_pix = list_msg->human_list[i].body_key_points_with_prob[j].x; y_pix = list_msg->human_list[i].body_key_points_with_prob[j].y;
                        prob = list_msg->human_list[i].body_key_points_with_prob[j].prob;

                        if (!std::isnan(x_pix) && !std::isnan(y_pix) && x_pix && y_pix)
                        {
                            /* Get an average of neighboring points coordinates for more precise x, y, z */
                            // x --> width, y --> height
                            int divisors = 0;
                            pcl::PointXYZRGBA p;

                            /* our point */
                            p = pPCL->at(x_pix, y_pix);

                            if (!std::isnan(p.x) && !std::isnan(p.y))
                            {
                                x = p.x; y = p.y; z = p.z;
                                x0 = x;
                                divisors++;

                                // if (j == 4)
                                //     ROS_INFO("j = %d: x_pix = %f, y_pix = %f --> p.x = %f, p.y = %f, p.z = %f", j, x_pix, y_pix, p.x, p.y, p.z);

                                /* debugging pointcloud */
                                if (j == 4)
                                    pPCL->at(x_pix, y_pix).r = 255;
                                pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                            }
                            /* P: our point, *: one of our point's neighbors
                                * * *
                                 ***
                                **P**
                                 ***
                                * * *
                            */
                            /* our point's 1st class neighbors */
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            /* some of our point's 2nd class neighbors */
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }

                            if (divisors)
                            {
                                x /= (double) divisors;
                                /* for edge cases */
                                if (x > UPPER_VARIATION_THRESH * x0 || x < LOWER_VARIATION_THRESH * x0) x = x0;
                            }

                            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || !x || !y || !z)
                            {
                                x = 0.0; y = 0.0; z = 0.0;
                                continue;
                            }

                            try
                            {
                                /* locally */
                                localTransform.setOrigin( tf::Vector3(x, y, z) );
                                tf::Quaternion localQuat(0, 0, 0, 1);
                                localTransform.setRotation(localQuat);
                                tfListener.setTransform( tf::StampedTransform(localTransform, ros::Time::now(), image_frame, getPoseBodyPartMappingBody25(j)) );

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
                                    
                                    humanMsg.body_key_points_with_prob[j].x = baseLinkTransform.getOrigin().x();
                                    humanMsg.body_key_points_with_prob[j].y = baseLinkTransform.getOrigin().y();
                                    humanMsg.body_key_points_with_prob[j].z = baseLinkTransform.getOrigin().z();
                                    humanMsg.body_key_points_with_prob[j].prob = prob;
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
        }
        else
        {
            static tf::TransformListener tfListener;
            static tf::StampedTransform baseLinkTransform;

            if (!tfSubtree)
            {
                try
                {
                    /* take the TF subtree that we want for the transformations */
                    tfListener.waitForTransform("base_link", image_frame, ros::Time(0), ros::Duration(TF_WAIT));
                    tfListener.lookupTransform("base_link", image_frame, ros::Time(0), baseLinkTransform);

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

                    for (uint32_t j = 0; j < 25; j++)
                    {
                        double x = 0.0, y = 0.0, z = 0.0, prob = 0.0, x_pix = 0.0, y_pix = 0.0, z_pix = 0.0, x0 = 0.0;

                        x_pix = list_msg->human_list[i].body_key_points_with_prob[j].x; y_pix = list_msg->human_list[i].body_key_points_with_prob[j].y; z_pix = list_msg->human_list[i].body_key_points_with_prob[j].z;
                        prob = list_msg->human_list[i].body_key_points_with_prob[j].prob;

                        if (!std::isnan(x_pix) && !std::isnan(y_pix) && x_pix && y_pix)
                        {
                            /* Get an average of neighboring points coordinates for more precise x, y, z */
                            // x --> width, y --> height
                            int divisors = 0;
                            pcl::PointXYZRGBA p;

                            /* our point */
                            p = pPCL->at(x_pix, y_pix);
                            if (!std::isnan(p.x) && !std::isnan(p.y))
                            {
                                x = p.x; y = p.y; z = p.z;
                                x0 = x;
                                divisors++;

                                // if (j == 4)
                                //     ROS_INFO("j = %d: x_pix = %f, y_pix = %f --> p.x = %f, p.y = %f, p.z = %f", j, x_pix, y_pix, p.x, p.y, p.z);

                                /* debugging pointcloud */
                                if (j == 4)
                                    pPCL->at(x_pix, y_pix).r = 255;
                                pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                            }
                            /* P: our point, *: one of our point's neighbors
                                * * *
                                 ***
                                **P**
                                 ***
                                * * *
                            */
                            /* our point's 1st class neighbors */
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-1*neighborhoodFactor >= 0 && x_pix-1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+1*neighborhoodFactor >= 0 && y_pix+1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-1*neighborhoodFactor, y_pix+1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+1*neighborhoodFactor >= 0 && x_pix+1*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-1*neighborhoodFactor >= 0 && y_pix-1*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+1*neighborhoodFactor, y_pix-1*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            /* some of our point's 2nd class neighbors */
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix >= 0 && y_pix < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix >= 0 && x_pix < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix-2*neighborhoodFactor >= 0 && x_pix-2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix+2*neighborhoodFactor >= 0 && y_pix+2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix-2*neighborhoodFactor, y_pix+2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }
                            if ((x_pix+2*neighborhoodFactor >= 0 && x_pix+2*neighborhoodFactor < IMG_PIXEL_WIDTH) && (y_pix-2*neighborhoodFactor >= 0 && y_pix-2*neighborhoodFactor < IMG_PIXEL_HEIGHT))    
                            {    
                                p = pPCL->at(x_pix+2*neighborhoodFactor, y_pix-2*neighborhoodFactor);
                                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                                {
                                    x += p.x;
                                    divisors++;

                                    /* debugging pointcloud */
                                    if (j == 4)
                                        pPCL->at(x_pix, y_pix).r = 255;
                                    pPCL->at(x_pix, y_pix).g = 255; pPCL->at(x_pix, y_pix).b = 0; pPCL->at(x_pix, y_pix).a = 255;
                                }
                            }

                            if (divisors)
                            {
                                x /= (double) divisors;
                                /* for edge cases */
                                if (x > UPPER_VARIATION_THRESH * x0 || x < LOWER_VARIATION_THRESH * x0) x = x0;
                            }

                            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || !x || !y || !z)
                            {
                                x = 0.0; y = 0.0; z = 0.0;
                                continue;
                            }

                            try
                            {
                                /* locally */
                                localTransform.setOrigin( tf::Vector3(x, y, z) );
                                tf::Quaternion localQuat(0, 0, 0, 1);
                                localTransform.setRotation(localQuat);
                                tfListener.setTransform( tf::StampedTransform(localTransform, ros::Time::now(), image_frame, getPoseBodyPartMappingBody25(j)) );

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
                                    
                                    humanMsg.body_key_points_with_prob[j].x = baseLinkTransform.getOrigin().x();
                                    humanMsg.body_key_points_with_prob[j].y = baseLinkTransform.getOrigin().y();
                                    humanMsg.body_key_points_with_prob[j].z = baseLinkTransform.getOrigin().z();
                                    humanMsg.body_key_points_with_prob[j].prob = prob;
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
        }

        /* for profiling */
        std_msgs::String stringMsg;
        stringMsg.data = strstream.str();
        robotFrameCoordsPub.publish(stringMsg);
        if (list_msg->num_humans)
            humanReceiverPub.publish(humanMsg);

        /* publish debugging's pointcloud */
        pointcloudDebugPub.publish(pPCL);

        pclMsg = false;
    }
    else
    {
        ros::spinOnce();
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
