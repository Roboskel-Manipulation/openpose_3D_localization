#include <keypoint_3d_matching.hpp>

std::string getPoseBodyPartMappingBody65(int idx){
    return POSE_BODY_65_BODY_PARTS.find(idx)->second;
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

keypoint_3d_matching_msgs::Keypoint3d_list keypointsStructure(std::vector<int> points_of_interest, std::string frame){
    keypoint_3d_matching_msgs::Keypoint3d point;
    keypoint_3d_matching_msgs::Keypoint3d_list points_v;

    for (int i=0; i<points_of_interest.size(); i++){
        point.name = getPoseBodyPartMappingBody65(points_of_interest[i]);
        point.points.header.frame_id = frame;
        points_v.keypoints.push_back(point);
    }
    return points_v;
}