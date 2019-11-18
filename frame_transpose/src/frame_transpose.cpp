#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <string>

std::string transform_frame, keypoint_name;

class Listener{
private:
	ros::Publisher pub;
	tf::TransformListener listener;
public:
	Listener();
	void callback(const keypoint_3d_matching_msgs::Keypoint3d_list msg);
};


Listener::Listener(){
	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::PointStamped>("/topic_transform", 1);
	tf::TransformListener listener;
}
void Listener::callback(const keypoint_3d_matching_msgs::Keypoint3d_list msg){
	ROS_INFO("Received the point");
	listener.waitForTransform(transform_frame, "camera_rgb_optical_frame", ros::Time(0), ros::Duration(10));
	geometry_msgs::PointStamped newPoint;
	for (short int i=0; i<msg.keypoints.size(); i++){
		if (msg.keypoints[i].name.compare(keypoint_name) == 0){
			ROS_INFO("Published the point");
			listener.transformPoint(transform_frame, ros::Time(0), msg.keypoints[i].points, "camera_rgb_optical_frame", newPoint);	
			pub.publish(newPoint);
		}
	}
}


int main(int argc, char** argv){
	ros::init (argc, argv, "frame_transpose");
	ros::NodeHandle n;
	n.param("frame_transpose/frame_name", transform_frame, std::string("/openpose_ros/human_list"));
	n.param("frame_transpose/keypoint_name", keypoint_name, std::string("/openpose_ros/human_list"));

	Listener ln;
	ROS_INFO("Created listener object");
	ros::Subscriber sub = n.subscribe("/keypoint_3d_matching", 1000, &Listener::callback, &ln);
	ros::spin();
}