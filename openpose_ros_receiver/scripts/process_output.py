#!/usr/bin/env python

import os
import glob
import re
import math
import time
	
'''
Get a list of keys from dictionary which has the given value
'''
def getKeysByValue(dictOfElements, valueToFind):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        if item[1] == valueToFind:
            listOfKeys.append(item[0])
    return  listOfKeys

if __name__ == "__main__":

    body_25_body_parts_dict = dict([ (0,  "Nose"), (1,  "Neck"), (2,  "RShoulder"), (3,  "RElbow"), (4,  "RWrist"),
                                     (5,  "LShoulder"), (6,  "LElbow"), (7,  "LWrist"), (8,  "MidHip"), (9,  "RHip"),
                                     (10, "RKnee"), (11, "RAnkle"), (12, "LHip"), (13, "LKnee"), (14, "LAnkle"),
                                     (15, "REye"), (16, "LEye"), (17, "REar"), (18, "LEar"), (19, "LBigToe"),
                                     (20, "LSmallToe"), (21, "LHeel"), (22, "RBigToe"), (23, "RSmallToe"), (24, "RHeel"),
                                     (25, "Background")
                                ])

    output_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/"
    output_subfolder = "take1/"
    output_folder_path = output_path + output_subfolder
    raw_output_file_prefix = "raw Tue Jan 15 14:21:2"   # to go from 14:21:20 to 14:21:29 (10 files)
    tfed_output_file_prefix = "tfed Tue Jan 15 14:21:"
    csv_folder_path = output_folder_path + "csv/"

    # create CSVs directory
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)

    # create CSVs
    for key, value in body_25_body_parts_dict.items():
        fp = open(csv_folder_path + value + ".csv", 'w')
        fp.close()
    
    # create our 3d report matrix: [BodyPart][x/y/z/prob][t1,...,t10,mean,variance] --> 25 * 4 * 12
    x, y, z = 25, 4, 12
    report_matrix = [ [ [ float('nan') for k in range(z) ] for j in range(y) ] for i in range(x) ]

    # access the files of the output directory
    file_counter = 0
    for file in os.listdir(output_folder_path):
        try:
            if os.path.isfile(os.path.join(output_folder_path, file)) and raw_output_file_prefix in file:
                # read from file
                with open(os.path.join(output_folder_path, file), 'r') as fp:
                    for cnt, line in enumerate(fp):
                        if "Body" not in line and line.strip():     # ignore header lines and empty lines
                            body_part = re.search('kp (.*):', line).group(1)
                            coords_and_prob = re.findall(r'\d+\.\d+', line)

                            # write in the appropriate CSV
                            with open(csv_folder_path + body_part + ".csv", 'a') as fp:
                                string = ""
                                coord_or_prob_idx = 0
                                for i in coords_and_prob:
                                    if string.strip():
                                        string = string + ","
                                    string = string + str(i)

                                    # print "m[{}][{}][{}] {}".format(getKeysByValue(body_25_body_parts_dict, body_part)[0], coord_or_prob_idx, file_counter, report_matrix[getKeysByValue(body_25_body_parts_dict, body_part)[0]][coord_or_prob_idx][file_counter])
                                    # fill report matrix
                                    # report_matrix[getKeysByValue(body_25_body_parts_dict, body_part)[0]][coord_or_prob_idx][file_counter] = float(i)

                                    coord_or_prob_idx = coord_or_prob_idx + 1

                                # report_matrix[getKeysByValue(body_25_body_parts_dict, body_part)[0]][coord_or_prob_idx+1][file_counter] = 0.0
                                # report_matrix[getKeysByValue(body_25_body_parts_dict, body_part)[0]][coord_or_prob_idx+2][file_counter] = 0.0

                                string = string + "\n"
                                fp.write(string)
            
            file_counter = file_counter + 1
        except Exception as e:
            raise e

    # # debugging
    # for i in range(x):
    #     for j in range(y):
    #         for k in range(z):
    #             print "m[{}][{}][{}] {}".format(i, j, k, report_matrix[i][j][k])
    #             if math.isnan(float(report_matrix[i][j][k])):
    #                 time.sleep(1)
