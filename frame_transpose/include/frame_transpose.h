#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

std::string topic_name, init_frame, target_frame;
boost::shared_ptr<tf::TransformListener> listener;
keypoint_3d_matching_msgs::Keypoint3d_list transformed_keypoints;

ros::Publisher pub;