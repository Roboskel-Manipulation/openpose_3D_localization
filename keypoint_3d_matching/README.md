# keypoint_3d_matching

## Description
A ROS package which matches the OpenPose output (2D pixels) to 3D PointCloud information. The output 3D coordinates are expressed in the camera frame.

## Configuration
* `points_of_interest`: A list of integers that declares the Openpose keypoints for which the 3D coordinates will be produced. The mapping between keypoints and integers is declared in `include/keypoint_3d_matching.yaml` and is compatible with the respective Openpose mapping as stated [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/src/openpose/pose/poseParameters.cpp). 
* `pointcloudEnable`: Flag for visual debugging in RViz.
* `pointcloud_topic`: Topic of the pointcloud provided by your camera. 
* `image_sensor_frame`: Topic of the RGB images provided by your camera.

## Run
- In a terminal run `roslaunch keypoint_3d_matching keypoint_3d_matching.launch`
