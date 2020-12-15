#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

float lWrist, lShoulder, threshold=0.02;
std_msgs::Bool msg;
ros::Publisher pub;

void check_keypoints_placement_callback (const keypoint_3d_matching_msgs::Keypoint3d_list::ConstPtr keypoint_msg){
	for (auto it = keypoint_msg->keypoints.begin(); it != keypoint_msg->keypoints.end(); ++it){
		if (it->name == "LWrist"){
			lWrist = it->points.point.z;
		}
		else if (it->name == "LShoulder"){
			lShoulder = it->points.point.z;
		}
	}
	ROS_INFO("%.3f, %.3f", lWrist, lShoulder);
	if (lWrist != 0 and lShoulder != 0){
		msg.data = lWrist - lShoulder > threshold ? true : false;
	}
	else{
		msg.data = true;
	}
	pub.publish(msg);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "check_keypoints_placement");
	ros::NodeHandle nh;

	
	pub = nh.advertise<std_msgs::Bool>("/check_keypoints_placement_topic", 100);

	ros::Subscriber sub = nh.subscribe("/transform_topic", 100, check_keypoints_placement_callback);

	ros::spin();
}