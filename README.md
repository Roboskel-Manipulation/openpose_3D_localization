# ROS packages for 3D keypoint estimation based on 2D Openpose pixels

## Description
A collection of ROS catkin packages that utilize the OpenPose library from [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose) in order to estimate the 3D position of the human in the space.

## Dependencies
This repo depends on:

* [Openpose wrapper](https://github.com/firephinx/openpose_ros)
* [Ar marker tracker](https://github.com/ros-perception/ar_track_alvar)
* [Camera utilities](https://github.com/Roboskel-Manipulation/manos_vision)
* [Tf constructor](https://github.com/gstavrinos/tf_based_on_ar_marker)

Note: The zed_wrapper package, which is also a dependency, has not been added to the package.xml, in order to allow for easier compilation. You will need to install it if you are planning to use a ZED camera.

Note_2: Make sure to clone the version of Openpose so that the wrapper is compatible. More info in [here](https://github.com/firephinx/openpose_ros).

## Packages:
* <b>keypoint_3d_matching</b>: Accepts Openpose pixels and a PointCloud and produces the 3D coordinates expressed in the camera frame.
* <b>keypoint_3d_matching_msgs</b>: Custom ROS mesages used by <b>keypoint_3d_matching</b>.
* <b>frame_transpose</b>: Transforms points to a different reference frame.
* <b>pipeline_launch</b>: Collection of launch files.

## Run
* In a terminal run `roslaunch pipeline_launch pipeline_launch.launch`
* Arguments:
	* sim: True if using rosbags and false if using camera
	* live_camera: True if running camera and false if using rosbag
	* marker: True if using marker

## System

Tested on:
* Ubuntu 16.04 / 18.04
* ROS Kinetic / Melodic
