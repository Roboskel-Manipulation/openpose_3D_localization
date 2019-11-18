#include "keypoint_3d_matching.hpp"


ros::Time timeStamp;
class Human3D{
private:
    std::vector<std::vector<int> > neighborhoodOffset;
    keypoint_3d_matching_msgs::Keypoint3d_list keypoints_v;
    int neighborhoodFactor;
public:
    Human3D(std::vector<std::vector<int> > int_vv, keypoint_3d_matching_msgs::Keypoint3d_list int_vv1n, int f);
    void pointCloudTopicCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_msgs);
    void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg);
};

Human3D::Human3D(std::vector<std::vector<int> > int_vv, keypoint_3d_matching_msgs::Keypoint3d_list int_vv1, int f)
:neighborhoodOffset(int_vv), keypoints_v(int_vv1), neighborhoodFactor(f){
}

void Human3D::pointCloudTopicCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_msg){
    if (humanListMsg){
        pclMsg = true;
        pPCL = pcl_msg;
        humanListMsg = false;
    }
    else{
        ros::spinOnce();
    }
}


void Human3D::humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& list_msg){
    if (pclMsg){
        humanListMsg = true;

        if (list_msg->num_humans == 0){
            ROS_INFO("No human detected in current frame.");
            return;
        }

        static tf::TransformListener tfListener;
        static tf::StampedTransform baseLinkTransform;

        // if (!tfSubtree){
        //     try{
        //         /* take the TF subtree that we want for the transformations */
        //         tfListener.waitForTransform("base_link", image_sensor_frame, ros::Time(0), ros::Duration(TF_WAIT));
        //         tfListener.lookupTransform("base_link", image_sensor_frame, ros::Time(0), baseLinkTransform);

        //         tfSubtree = true;
        //     }
        //     catch (tf::TransformException &ex){
        //         ROS_ERROR("%s", ex.what());
        //     }
        // }


        for (short int i=0; i<points_of_interest.size(); i++){
            double x=0.0, y=0.0, z=0.0;
            double x_pix = list_msg->human_list[0].body_key_points_with_prob[points_of_interest[i]].x;
            double y_pix = list_msg->human_list[0].body_key_points_with_prob[points_of_interest[i]].y;
            double prob = list_msg->human_list[0].body_key_points_with_prob[points_of_interest[i]].prob;

            if (!std::isnan(x_pix) && !std::isnan(y_pix) && x_pix && y_pix){

                pcl::PointXYZRGBA p = pPCL->at(x_pix, y_pix);
                timeStamp = ros::Time::now();
                double x0 = p.x;
                int divisors = 0;

                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) || !p.x || !p.y || !p.z){
                    x = 0.0; y = 0.0; z = 0.0;
                    continue;
                }
                
                // static tf::Transform localTransform;
                x = p.x;
                y = p.y;
                z = p.z;

                for (short int j=0; j<neighborhoodOffset.size(); j++){
                    if ((x_pix+neighborhoodFactor*neighborhoodOffset[j][1] >= 0 && x_pix+neighborhoodFactor*neighborhoodOffset[j][1] < IMG_PIXEL_WIDTH) && (y_pix+neighborhoodFactor*neighborhoodOffset[j][0] >= 0 && y_pix+neighborhoodFactor*neighborhoodOffset[j][0] < IMG_PIXEL_HEIGHT)){
                        p = pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]);
                        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)){
                            x += p.x;
                            divisors++;

                            /* debugging pointcloud */
                            if (pointcloudEnable){
                                if (neighborhoodOffset[j][0] == 0 && neighborhoodOffset[j][1] == 0){
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).r = RED_KEYPOINT;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).g = GREEN_KEYPOINT;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).b = BLUE_KEYPOINT;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).a = 255;
                                }
                                else{
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).r = RED;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).g = GREEN;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).b = BLUE;
                                    pPCL->at(x_pix+neighborhoodFactor*neighborhoodOffset[j][1], y_pix+neighborhoodFactor*neighborhoodOffset[j][0]).a = 255;   
                                }
                            }
                        }
                    }
                }
                if (divisors){
                    x /= (double) divisors;
                    /* for edge cases */
                    if (x > UPPER_VARIATION_THRESH * x0 || x < LOWER_VARIATION_THRESH * x0) 
                        x = x0;
                }

                keypoints_v.keypoints[i].points.point.x = x;
                keypoints_v.keypoints[i].points.point.y = y;
                keypoints_v.keypoints[i].points.point.z = z;
                keypoints_v.keypoints[i].points.header.stamp = timeStamp;
                // try{
                //     // locally 
                //     localTransform.setOrigin( tf::Vector3(x, y, z) );
                //     tf::Quaternion localQuat(0, 0, 0, 1);
                //     localTransform.setRotation(localQuat);
                //     tfListener.setTransform(tf::StampedTransform(localTransform, ros::Time::now(), image_sensor_frame, getPoseBodyPartMappingBody25(points_of_interest[i])) );

                //     // LOOKUP LOCAL TRANSFORM 
                //     tfListener.waitForTransform("base_link", getPoseBodyPartMappingBody25(points_of_interest[i]), ros::Time(0), ros::Duration(TF_WAIT));
                //     tfListener.lookupTransform("base_link", getPoseBodyPartMappingBody25(points_of_interest[i]), ros::Time(0), baseLinkTransform);

                //     // log 
                //     if (!std::isnan(baseLinkTransform.getOrigin().x()) && !std::isnan(baseLinkTransform.getOrigin().y()) && !std::isnan(baseLinkTransform.getOrigin().z())){
                //         // process transform's timestamp
                //         ros::Time now = ros::Time::now(), transformStamp = baseLinkTransform.stamp_;
                //         ros::Duration timeDiff = now - transformStamp;
                //         // check if we are dealing with a frame old enough (e.g. > 2 sec) to be considered unreliable
                //         if (timeDiff > ros::Duration(RELIABILITY_THRESHOLD)){
                //             ROS_WARN("Transform older than %f seconds detected", RELIABILITY_THRESHOLD);
                //             continue;
                //         }
                //         keypoints_v.keypoints[i].points.x = baseLinkTransform.getOrigin().x();
                //         keypoints_v.keypoints[i].points.y = baseLinkTransform.getOrigin().y();
                //         keypoints_v.keypoints[i].points.z = baseLinkTransform.getOrigin().z();
                //     }
                // }
                // catch (tf::TransformException &ex){
                //     ROS_ERROR("%s", ex.what());
                //     continue;
                // }
            }
        }
        
        /* for profiling */
        humanReceiverPub.publish(keypoints_v);
        for (short int i=0; i<keypoints_v.keypoints.size(); i++){
            keypoints_v.keypoints[i].points.point.x = 0;
            keypoints_v.keypoints[i].points.point.y = 0;
            keypoints_v.keypoints[i].points.point.z = 0;
        }

        /* publish debugging's pointcloud */
        if (pointcloudEnable)
            pointcloudDebugPub.publish(pPCL);

        pclMsg = false;
    }
    else{
        ros::spinOnce();
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "keypoint_3d_matching");
    ros::NodeHandle nh;
    
    int queue_size, factor;
    std::string image_sensor_frame, human_list_topic, pointcloud_topic, pointcloud_topic_debug, robot_frame_coords_str_topic, robot_frame_coords_msg_topic;

    nh.param("keypoint_3d_matching/human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("keypoint_3d_matching/pointcloud_topic", pointcloud_topic, std::string("/zed/point_cloud/cloud_registered"));
    nh.param("keypoint_3d_matching/pointcloud_topic_debug", pointcloud_topic_debug, std::string("/zed/point_cloud/cloud_registered_debug"));
    nh.param("keypoint_3d_matching/image_sensor_frame", image_sensor_frame, std::string("/zed_left_camera_frame"));
    nh.param("keypoint_3d_matching/keypoint_3d_matching_topic", robot_frame_coords_msg_topic, std::string("/openpose_ros_receiver/robot_frame_coords_msg"));
    nh.param("keypoint_3d_matching/queue_size", queue_size, 2);
    nh.param("keypoint_3d_matching/points_of_interest", points_of_interest, std::vector<int>(0));
    nh.param("keypoint_3d_matching/neighborhoodFactor", factor, 1);
    nh.param("keypoint_3d_matching/pointcloudEnable", pointcloudEnable, false);

    /* Initialize Global Variables */
    pclMsg = false; 
    humanListMsg = true;


    // Create the structure of the final message based on the points of interest
    keypoint_3d_matching_msgs::Keypoint3d_list points_v = keypointsStructure(points_of_interest, image_sensor_frame);

    // Create a vector containing the neighborhood offsets
    std::vector<std::vector<int> > neighborhood_v = neighborhood_vector();

    
    /* Publish the 3D coordinates of the points of interest and optionally the debugging pointcloud */
    humanReceiverPub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>(robot_frame_coords_msg_topic, queue_size);
    if (pointcloudEnable){
        pointcloudDebugPub =nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(pointcloud_topic_debug, queue_size);
    } 


    Human3D obj(neighborhood_v, points_v, factor);

    /* Subscribe to the output topic of the 2D openpose pipeline */

    ros::Subscriber subPointcloud = nh.subscribe(pointcloud_topic, queue_size, &Human3D::pointCloudTopicCallback, &obj);
    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, queue_size, &Human3D::humanListCallback, &obj);

    ros::spin();


    return 0;
}
