# openpose_3D_localization

## Description
A collection of ROS packages that estimates the 3D position of human body joints in a requested reference frame. It is based on  [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) 2D joint information  

## Dependencies
This repo depends on:

* [OpenPose wrapper](https://github.com/firephinx/openpose_ros)
* [AR marker tracker](https://github.com/ros-perception/ar_track_alvar)
* [Camera utilities](https://github.com/Roboskel-Manipulation/manos_vision)
* [TF constructor](https://github.com/gstavrinos/tf_based_on_ar_marker)

## Packages:
* <b>keypoint_3d_matching</b>: A ROS package which matches the OpenPose output (2D pixels) to 3D PointCloud information. The output 3D coordinates are expressed in the camera frame.
* <b>keypoint_3d_matching_msgs</b>: Custom ROS messages used by <b>keypoint_3d_matching</b>.
* <b>frame_transpose</b>: Transforms points to a different reference frame.
* <b>pipeline_launch</b>: Collection of launch files.

## Run
* In a terminal run `roslaunch pipeline_launch pipeline_launch.launch`
* Arguments:
    * sim: Set [use_sim_time](http://wiki.ros.org/Clock) (True if playing running a rosbag. Make sure you play the rosbag including the --clock flag.)
    * live_camera: True if running a camera
    * marker: True if using marker

## System

Tested on:
* Ubuntu 16.04 / 18.04
* ROS Kinetic / Melodic
