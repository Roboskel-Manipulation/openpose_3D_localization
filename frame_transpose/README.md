# frame_transpose

# Description
A ROS package for transforming 3D points from an initial frame to a target frame.

# Run

In config/frame_transpose.yaml specify the following:
- `topic_name`: topic in which the initial points are published
- `init_frame`: the frame in which the initial points are expressed to
- `target_frame`: the frame in which we want to express the points

In a terminal run:

    roslaunch frame_transpose frame_transpose
