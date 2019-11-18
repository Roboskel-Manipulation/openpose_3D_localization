#!/usr/bin/env python

import rospy
from openpose_ros_receiver_msgs.msg import Keypoints_v
from openpose_ros_receiver_msgs.msg import OpenPoseReceiverHuman

count = 0

def callback(key):
	global count 
	count += 1
	print (count)

def main():
	rospy.init_node("listener", anonymous=True)
	rospy.Subscriber("/openpose_ros_receiver/robot_frame_coords_msg", OpenPoseReceiverHuman, callback)
	rospy.spin()

if __name__=="__main__":
	main()