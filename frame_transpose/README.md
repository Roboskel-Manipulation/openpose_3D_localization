# frame_transpose

## Description
A ROS package for transforming 3D points from an initial frame to a target frame.

## Configuration 

In <b> config/frame_transpose.yaml </b> specify the following:
- `topic_name`: topic in which the initial points are published
- `init_frame`: the frame in which the initial points are expressed to
- `target_frame`: the frame in which we want to express the points

## Run
In a terminal run `roslaunch frame_transpose frame_transpose.launch`
