#include "openpose_ros_receiver.hpp"

void writeSkeletonToFile(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    /* log skeletons to file */
    std::ofstream outfile;
    outfile.open("/home/gkamaras/openpose_ros_receiver.txt");

    for (uint32_t i = 0; i < msg->num_humans; i++)
    {
        outfile << "Body keypoints:" << std::endl;
        for (uint32_t j = 0; j < msg->human_list[i].num_body_key_points_with_non_zero_prob; j++)
            outfile << "kp " << j << ": x=" <<  msg->human_list[i].body_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].body_key_points_with_prob[j].y
                    << " z=" <<  msg->human_list[i].body_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].body_key_points_with_prob[j].prob << std::endl;
        
        outfile << "Face keypoints:" << std::endl;
        for (uint32_t j = 0; j < msg->human_list[i].num_face_key_points_with_non_zero_prob; j++)
            outfile << "kp " << j << ": x=" <<  msg->human_list[i].face_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].face_key_points_with_prob[j].y
                    << " z=" <<  msg->human_list[i].face_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].face_key_points_with_prob[j].prob << std::endl;
        
        outfile << "Right hand keypoints:" << std::endl;
        for (uint32_t j = 0; j < msg->human_list[i].num_right_hand_key_points_with_non_zero_prob; j++)
            outfile << "kp " << j << ": x=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].y
                    << " z=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].right_hand_key_points_with_prob[j].prob << std::endl;
        
        outfile << "Left hand keypoints:" << std::endl;
        for (uint32_t j = 0; j < msg->human_list[i].num_left_hand_key_points_with_non_zero_prob; j++)
            outfile << "kp " << j << ": x=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].x << " y=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].y
                    << " z=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].z << " prob=" <<  msg->human_list[i].left_hand_key_points_with_prob[j].prob << std::endl;
    
        outfile << std::endl << std::endl;
    }

    outfile.close();
}

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

    writeSkeletonToFile(msg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "openpose_ros_receiver_node");

    ros::NodeHandle nh;
    std::string human_list_topic;
    int human_list_queue_size;
    nh.param("human_list_topic", human_list_topic, std::string("/openpose_ros/human_list"));
    nh.param("human_list_queue_size", human_list_queue_size, 10);

    ros::Subscriber subHumanList = nh.subscribe(human_list_topic, human_list_queue_size, humanListCallback);

    ros::spin();

    return 0;
}