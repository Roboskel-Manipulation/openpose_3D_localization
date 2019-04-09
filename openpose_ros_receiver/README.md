# openpose_ros_receiver

Receive the OpenPose output's skeletons and transform their keypoints coordinates from the camera coordinate frame to the robot coordinate frame.

## Important Note

This software is a product of on-going research and has been developed and tested with only one human in the robot workspace environment. Its adaptation for an environment with multiple humans is left open for future research.

## Dependencies

* **zed-ros-wrapper**: https://github.com/stereolabs/zed-ros-wrapper
* **manos_description**: https://github.com/Roboskel-Manipulation/manos
* **manos_vision**: https://github.com/Roboskel-Manipulation/manos_vision

## Run

```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros_receiver openpose_ros_receiver.launch
```
