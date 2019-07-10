#!/usr/bin/env python
# This is a script that takes a series of rosbags as input
# and runs OpenPose in order to create a new rosbag that
# contains all the topics from the original rosbags with the
# addition of the openpose output.
# # NOT THOROUGHLY TESTED, but it is working for us.
import yaml
from rosbag import Bag
import subprocess, shlex
import time, os, signal, sys

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\033[91m" + "Need exactly at least two arguments (the folder that contains\
         the rosbags, and extra topic names that will be recorded).\nExiting...\n" + "\033[0m")
        exit()
    print("\033[92m" + "Welcome to the Batch Rosbag Generator Script!\n" + "\033[0m")
    rosbags = [file for file in os.listdir(sys.argv[1]) if file.endswith(".bag")]
    if rosbags:
        command = "roscore"
        command = shlex.split(command)
        subprocess.Popen(command)
        time.sleep(1)
        print("\033[93m" + "roscore is now running!\n" + "\033[0m")
    else:
        print("\033[91m" + "Could not find any .bag files in " + sys.argv[1] + "\n" + "\033[0m")
        exit()
    if sys.argv[1].endswith(os.sep):
        sys.argv[1] = sys.argv[1][:-1]
    target_folder = sys.argv[1] + os.sep + "script_generated"
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
    for rosbag in rosbags:
        # COMMENT OUT the
        # <include file="$(find manos_vision)/launch/orbbec_astra.launch"/>
        # from openpose_ros_orbbec.launch, in line 4.
        command = "roslaunch openpose_utils_launch openpose_ros_orbbec.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        time.sleep(10)
        print("\033[93m" + "OpenPose is now running!\n" + "\033[0m")

        original_rosbag_location = sys.argv[1] + os.sep + rosbag

        final_name = sys.argv[1] + os.sep + "script_generated/" + rosbag[:-4] + "_openpose.bag"
        extra_topics = " ".join(sys.argv[2:])
        bag_info = yaml.load(Bag(original_rosbag_location, "r")._get_yaml_info())

        command = "rosbag play " + original_rosbag_location
        command = shlex.split(command)
        rosbag_process = subprocess.Popen(command)
        print("\033[93m" + "Started playing rosbag " + rosbag + "...\n" + "\033[0m")

        command = "rosbag record -O " + final_name + " " + extra_topics
        command = shlex.split(command)
        rosbag_process = subprocess.Popen(command)
        print("\033[93m" + "Started recording...\n" + "\033[0m")

        time.sleep(bag_info["duration"] + 30)

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            if(node != "/rosout"):
                os.system("rosnode kill "+ node)

    os.system("killall -9 rosmaster")
    time.sleep(2)
    print("\033[92m" + "Done. Good bye!\n" + "\033[0m")

#Example usage:
#python rosbag_openpose_addition.py /home/gstavrinos/rosbags/ /acoustic_magic_doa/data_raw /audio /camera/depth/image_raw /camera/rgb/image_raw /scan /openpose_ros/human_list
