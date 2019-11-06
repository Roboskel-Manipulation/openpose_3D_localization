#include <openpose_3D_producer.hpp>

std::string getPoseBodyPartMappingBody25(int idx){
    return POSE_BODY_25_BODY_PARTS.find(idx)->second;
}

std::vector<std::vector<int> > neighborhood_vector(){
    std::vector<int> int_v;
    std::vector<std::vector<int> > int_vv;

    for (short int i=-2; i<3; i++){
        for (short int j=-2; j<3; j++){
            if (abs(i)+abs(j) != 3){
                int_v.push_back(i);
                int_v.push_back(j);
                int_vv.push_back(int_v);
                int_v.clear();
            }
        }
    }
    return int_vv;
}

openpose_ros_receiver_msgs::Keypoints_v keypointsStructure(std::vector<int> points_of_interest){
    openpose_ros_receiver_msgs::Keypoints point;
    openpose_ros_receiver_msgs::Keypoints_v points_v;

    for (int i=0; i<points_of_interest.size(); i++){
        point.name = getPoseBodyPartMappingBody25(points_of_interest[i]);
        points_v.keypoints.push_back(point);
    }
    return points_v;
}