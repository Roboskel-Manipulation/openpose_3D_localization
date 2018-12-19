#include "openpose_ros_receiver.hpp"

void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    // for (uint32_t i = 0; i < msg->num_humans; i++)
    // {
    //     for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
    //         ROS_INFO("Body kp: %f %f %f", msg->human_list[i].body_key_points_with_prob[j].x, msg->human_list[i].body_key_points_with_prob[j].y, msg->human_list[i].body_key_points_with_prob[j].z);
    //     for (uint32_t j = 0; j < msg->human_list[i].num_face_key_points_with_non_zero_prob; j++)
    //         ROS_INFO("%Face kp: f %f %f", msg->human_list[i].face_key_points_with_prob[j].x, msg->human_list[i].face_key_points_with_prob[j].y, msg->human_list[i].face_key_points_with_prob[j].z);
    //     for (uint32_t j = 0; j < msg->human_list[i].num_right_hand_key_points_with_non_zero_prob; j++)
    //         ROS_INFO("Right hand kp: %f %f %f", msg->human_list[i].right_hand_key_points_with_prob[j].x, msg->human_list[i].right_hand_key_points_with_prob[j].y, msg->human_list[i].right_hand_key_points_with_prob[j].z);
    //     for (uint32_t j = 0; j < msg->human_list[i].num_left_hand_key_points_with_non_zero_prob; j++)
    //         ROS_INFO("Left hand kp: %f %f %f", msg->human_list[i].left_hand_key_points_with_prob[j].x, msg->human_list[i].left_hand_key_points_with_prob[j].y, msg->human_list[i].left_hand_key_points_with_prob[j].z);
    // }

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    for (uint32_t i = 0; i < msg->num_humans; i++)
    {
        for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
        {
            if (!std::isnan(msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].z))
            {
                transform.setOrigin( tf::Vector3(msg->human_list[i].body_key_points_with_prob[j].x, msg->human_list[i].body_key_points_with_prob[j].y, msg->human_list[i].body_key_points_with_prob[j].z) );
                tf::Quaternion q(0, 0, 0, 1);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(j)));
            }
        }
    }

    writeSkeletonToFile(msg);
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
            outfile << "Body keypoints:" << std::endl;
            for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++) {
                if (!std::isnan(msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].z))
                {
                    outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" <<  msg->human_list[i].body_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].body_key_points_with_prob[j].y
                            << " z=" <<  msg->human_list[i].body_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;
                }
            }
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

std::string getPoseBodyPartMappingBody25(unsigned int idx) {
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}