#include "frame_transpose.h"

void transform_callback(const keypoint_3d_matching_msgs::Keypoint3d_list::ConstPtr camera_keypoints){
	// Get transform and throw exception if not available after some time (3 secs)
	try{
		listener->waitForTransform(target_frame, init_frame, ros::Time(0), ros::Duration(3));
	}
	catch(tf::TransformException& ex){
		ROS_ERROR_STREAM("Error: " << ex.what());
	}

	// Transform the point and construct the message
	keypoint_3d_matching_msgs::Keypoint3d transformed_point;
	geometry_msgs::PointStamped point;
	for (auto it = camera_keypoints->keypoints.begin(); it != camera_keypoints->keypoints.end(); ++it){
		transformed_point.name = it->name;
		if (it->points.point.x == 0 and 
			it->points.point.y == 0 and 
			it->points.point.z == 0){
			transformed_point.points.point.x = 0;
			transformed_point.points.point.y = 0;
			transformed_point.points.point.z = 0;
			transformed_point.points.header.stamp = it->points.header.stamp;
			transformed_point.points.header.frame_id = target_frame;
		}
		else{
			listener->transformPoint(target_frame, ros::Time(0), it->points, init_frame, point);
			transformed_point.points.point.x = point.point.x;
			transformed_point.points.point.y = point.point.y;
			transformed_point.points.point.z = point.point.z;
			transformed_point.points.header.stamp = it->points.header.stamp;
			transformed_point.points.header.frame_id = target_frame;
		}
		transformed_keypoints.keypoints.push_back(transformed_point);
	}
	pub.publish(transformed_keypoints);
	transformed_keypoints.keypoints.clear();
}

int main(int argc, char** argv){
	ros::init (argc, argv, "frame_transpose");
	ros::NodeHandle nh;
	nh.param("frame_transpose/topic_name", topic_name, std::string("keypoint_3d_matching"));
	nh.param("frame_transpose/init_frame", init_frame, std::string("camera_rgb_optical_frame"));
	nh.param("frame_transpose/target_frame", target_frame, std::string("base_link"));
	listener = boost::make_shared<tf::TransformListener>();

	pub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>("/transform_topic", 1000);
	ros::Subscriber sub = nh.subscribe(topic_name, 1000, transform_callback);

	ros::spin();
}