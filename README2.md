# ROS packages for 3D keypoint estimation based on 2D keypoint pixels

A collection of ROS catkin packages that utilize the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose in order to detect human presence near a robotic arm workspace.

This repo depends on:

* https://github.com/firephinx/openpose_ros
* https://github.com/ros-perception/ar_track_alvar
* https://github.com/Roboskel-Manipulation/manos_vision
* https://github.com/gstavrinos/tf_based_on_ar_marker


Note: The zed_wrapper package, which is also a dependency, has not been added to the package.xml, in order to allow for easier compilation. You will need to install it if you are planning to use a ZED camera.

Note_2: Make sure to clone the version of Openpose so that the wrapper is compatible. More info in https://github.com/firephinx/openpose_ros.

## General instructions

Run the whole_pipeline_* launch files to launch the pipeline. Replace the * with the camera of your choice. Currently three camera drivers are supported. ASUS XTION, ORBBEC ASTRA and ZED.

## Instructions for Roboskel

Run the launch file openpose_launch.launch file where specific arguments exist.

## Arguments:
* **sim**: True if you want the ROS API to get times from the topic /clock and not the system clock (default true).
* **live_camera**: True if you want to load the camera driver (default false).
* **manos_tf**: True if you want to load the tf of manos (default false).

Generally you should only need to directly run the launch files located inside the openpose_utils_launch package.
Use the openpose_ros_* launch files, to start the openpose_ros wrapper along with the camera driver of your choice.
For the whole (experimental) pipeline you should use the whole_pipeline_* launch files, replacing * with the camera of your choice.
The launch files that end with *_with_marker should be used only under certain circimstances that do not apply to a wide audience, but rather specifically to our lab (dependency on https://github.com/Roboskel-Manipulation/manos).

If you want to be able to see the openpose output, edit the openpose_ros_* launch file, and change the `display_output` parameter to `true`.

The launch files are configured to play by default using data from rosbags. If you want to run the procedures live, use the `live:=true` and `use_sim_time:=false` params.

## Contents

* **keypoints_3d_matching**: Compute of the 3D coordinates of the desired keypoints.
* **keypoints_3d_matching_msgs**: Messages for the keypoints_3d_matching package.
* **openpose_utils_launch**: Launch files for all the above.

## System

Tested on:
* ROS Indigo / Kinetic / Melodic
* openpose_ros commit #105e950
* Openpose commit 
