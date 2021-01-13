# keypoint_3d_matching

## Description
A ROS package which matches the OpenPose output (2D pixels) to 3D PointCloud information. The output 3D coordinates are expressed in the camera frame.

- src/keypoint_3d_matching.cpp: Main functionality for the 3D matching
- src/functions.cpp: Functions used by keypoint_3d_matching.cpp
- config/keypoint_3d_matching.yaml: Parameters used by the keypoint_3d_matching ROS node
- include/keypoint_3d_matching.h: Header files and variable declarations
- launch/keypoint_3d_matching.launch: Launch file for keypoint_3d_matching ROS node

## Configuration
* `points_of_interest`: A list of integers that declares the Openpose keypoints for which the 3D coordinates will be produced. The mapping between keypoints and integers is declared in `include/keypoint_3d_matching.yaml` and is compatible with the respective Openpose mapping as stated [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/src/openpose/pose/poseParameters.cpp). 
* `pointcloudEnable`: Flag for debugging (publishing to the `pointcloud_topic_debug`)
* `pointcloud_topic`: Topic of the pointcloud provided by your camera. 
* `image_sensor_frame`: Topic of the RGB images provided by your camera.

## Run
- In a terminal run `roslaunch keypoint_3d_matching keypoint_3d_matching.launch`
