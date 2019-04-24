# OpenPose ROS wrapper for Human-Robot Collaboration

A collection of ROS catkin package that utilize the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose in order to detect human presence near a robotic arm workspace and provide the robot's motion planner with the input which is necessary to guarantee safe Human-Robot Collaboration.

![alt text](./doc/img/TheCompleteHumanBodyAsAnObstacleSoftwarePipeline.png?raw=true)

## Contents

* **openpose_ros**: The core functionality of the OpenPose ROS wrapper. Take a stream of images and in each image perform human pose recognition, using a human pose model (COCO_18, BODY_25). Describe each detected human with a skeleton made up of certain detected keypoints.
* **openpose_ros_msgs**: The messages used to publish the human skeletons of the OpenPose output to a ROS environment.
* **openpose_ros_receiver**: Receive the OpenPose output's skeletons and transform their keypoints coordinates from the camera coordinate frame to the robot coordinate frame.
* **openpose_ros_receiver_msgs**: The messages used to publish the openpose_ros_receiver output (human body skeletons with keypoint coordinates in the robot coordinate frame) to a ROS environment. 
* **openpose_ros_avoid**: Receive human body skeletons based on the BODY_25 human pose model and insert them as obstacles in a robotic arm's workspace.

## System

Tested on:
* Software:
    * Ubuntu 14.04 / Ubuntu 16.04
    * ROS Indigo / Kinetic
    * MoveIt! 1.0.0
    * CUDA 8.0
    * cuDNN 5.1 / cuDNN 6.0
    * OpenCV 3.3 / OpenCV 3.4
    * ZED SDK 2.7.1 (should do fine with any version ≥ 2.3)
* Hardware:
    * UR3 robotic arm
    * Stereolabs ZED camera
    * ASUS Xtion camera
    * Orbbec Astra Pro camera
