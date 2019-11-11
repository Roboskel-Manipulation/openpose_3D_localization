#!/usr/bin/env python

import rospy
from openpose_ros_receiver_msgs.msg import Keypoints_v

count = 0

def callback():
	global count 
	count += 1
	std::cout << count << std::endl;

def main():
	rospy.init_node("listener" anonymous=True)
	rospy.Subscriber("/openpose_ros_receiver/robot_frame_coords_msg", Keypoints_v, callback)
	rospy.spin()

if __name__=="__main__":
	main()