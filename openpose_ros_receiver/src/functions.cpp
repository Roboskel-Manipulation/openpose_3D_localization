#include "openpose_ros_receiver.hpp"

void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
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
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", getPoseBodyPartMappingBody25(j)/*+"_"+std::to_string(i)*/) );
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
            tf::TransformListener listener;
            tf::StampedTransform transform;
            ros::Rate rate(5.0);

            outfile << "Body keypoints:" << std::endl;
            for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
            {
                if (!std::isnan(msg->human_list[i].body_key_points_with_prob[j].x) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].y) && !std::isnan(msg->human_list[i].body_key_points_with_prob[j].z))
                {
                    rate.sleep();
                    try
                    {
                        listener.waitForTransform("gripper_link", "zed_left_camera_frame", ros::Time(0), ros::Duration(3.0));
                        listener.lookupTransform("gripper_link", "zed_left_camera_frame", ros::Time(0), transform);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                    }

                    outfile << "kp " << getPoseBodyPartMappingBody25(j) << ": x=" << transform.getOrigin().x() << " y=" <<  transform.getOrigin().y()
                            << " z=" <<  transform.getOrigin().y() << " prob=" <<  msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;
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

std::string getPoseBodyPartMappingBody25(unsigned int idx)
{
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}