# openpose_ros_avoid

Receive human body skeletons based on the BODY_25 human pose model and insert them as obstacles in a robotic arm's workspace.

## Important Note

This software is a product of on-going research and has been developed and tested with only one human in the robot workspace environment. Its adaptation for an environment with multiple humans is left open for future research.

## Dependencies

* **manos_bringup**, **manos_gazebo**, **manos_moveit_config**: https://github.com/Roboskel-Manipulation/manos

## Run

Open 5 terminal windows.

### Physical (UR3 Hardware)

Terminal 1: 
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros_receiver openpose_ros_receiver.launch
```

Terminal 2: 
```bash
roslaunch openpose_ros_avoid openpose_ros_avoid.launch
```

Terminal 3: 
```bash
roslaunch manos_bringup manos_bringup.launch robot_ip:=ROBOT_IP_ADDRESS
```

Terminal 4: 
```bash
roslaunch manos_moveit_config manos_planning_execution.launch
```

Terminal 5: 
```bash
roslaunch manos_moveit_config moveit_rviz.launch config:=true
```

### Simulation (UR3 Gazebo)

Terminal 1: 
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros_receiver openpose_ros_receiver.launch
```

Terminal 2: 
```bash
roslaunch openpose_ros_avoid openpose_ros_avoid.launch
```

Terminal 3: 
```bash
roslaunch manos_gazebo manos_gazebo.launch
```

Terminal 4: 
```bash
roslaunch manos_moveit_config manos_planning_execution.launch sim:=true
```

Terminal 5: 
```bash
roslaunch manos_moveit_config moveit_rviz.launch config:=true
```