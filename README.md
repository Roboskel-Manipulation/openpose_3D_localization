# Openpose ROS packages for 3D keypoint estimation based on 2D keypoint pixels

A collection of ROS catkin packages that utilize the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose in order to detect human presence near a robotic arm workspace.

This repo depends on:

* https://github.com/firephinx/openpose_ros
* https://github.com/ros-perception/ar_track_alvar
* https://github.com/Roboskel-Manipulation/manos_vision
* https://github.com/gstavrinos/tf_based_on_ar_marker


Note: The zed_wrapper package, which is also a dependency, has not been added to the package.xml, in order to allow for easier compilation. You will need to install it if you are planning to use a ZED camera.

## General instructions

Run the whole_pipeline_* launch files to launch the pipeline. Replace the * with the camera of your choice. Currently three camera drivers are supported. ASUS XTION, ORBBEC ASTRA and ZED.

## Instructions for Roboskel.

Run the launch file openpose_sole.launch file if you want to use Openpose solely and openpose_manos.launch file if you want to use Openpose along with Manos.

## Arguments:
* **sim**: True if you want the ROS API to get times from the topic /clock and not the system clock (default true).
* **live_camera**: True if you want to load the camera driver (default false).
* **manos_tf**: True if you want to load the tf of manos (default false).

If you want to be able to see the openpose output, edit the openpose_ros_* launch file, and change the `display_output` parameter to `true`.

## Contents

* **keypoints_3d_matching**: Compute of the 3D coordinates of the desired keypoints.
* **keypoints_3d_matching_msgs**: Messages for the keypoints_3d_matching package.
* **openpose_utils_launch**: Launch files for all the above.

## System

Tested on:
* ROS Indigo / Kinetic / Melodic
* openpose_ros commit #105e950
