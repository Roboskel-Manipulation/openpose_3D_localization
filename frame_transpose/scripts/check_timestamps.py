#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import keyboard

depth_times, rgb_times = [], []

def rgb_callback(msg):
	global rgb_times
	rgb_times.append(msg.header.stamp.to_sec())

def depth_callback(msg):
	global depth_times
	depth_times.append(msg.header.stamp.to_sec())


def main():
	global rgb_times, depth_times
	rospy.init_node("check_timestamps")
	rospy.Subscriber("/camera/depth/image", Image, depth_callback)
	rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)

	rospy.spin()
	while True:
		try:
			key = raw_input('Give a number')
			if key == 'q':
				print (len(depth_times), len(rgb_times))
				for i in range(min(len(depth_times), len(rgb_times))):
					print (abs(depth_times[i]-rgb_times[i]))
				break
		except Exception as err:
			print (err)

main()