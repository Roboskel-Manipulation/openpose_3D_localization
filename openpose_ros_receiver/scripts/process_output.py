#!/usr/bin/env python

import os
import re
import math
import time
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Get a list of keys from dictionary which has the given value
def getKeysByValue(dictOfElements, valueToFind):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        if item[1] == valueToFind:
            listOfKeys.append(item[0])
    return  listOfKeys

# Define a function for a histogram
def histogram(data, x_label, y_label, title, directory):
    fig, ax = plt.subplots()
    ax.hist(np.array(data)[~np.isnan(np.array(data))], color = '#539caf')
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    plt.savefig(directory+title+".png")
    plt.close(fig)

# Define a function for a 3D scatterplot
def scatterplot(x, y, z, x_label, y_label, z_label, title, directory):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='r', marker='o')
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_zlabel(z_label)
    ax.set_title(title)
    plt.savefig(directory+title+".png")
    plt.close(fig)

# Define a function for plotting error
def errorbar(x, y, x_label, y_label, title, directory, x_lim_min=0, x_lim_max=1):
    dy = (x_lim_min + x_lim_max) / 2
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.errorbar(x, y, yerr=dy, fmt='.k')
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    plt.savefig(directory+title+".png")
    plt.close()

if __name__ == "__main__":

    body_25_body_parts_dict = dict([ (0,  "Nose"), (1,  "Neck"), (2,  "RShoulder"), (3,  "RElbow"), (4,  "RWrist"),
                                     (5,  "LShoulder"), (6,  "LElbow"), (7,  "LWrist"), (8,  "MidHip"), (9,  "RHip"),
                                     (10, "RKnee"), (11, "RAnkle"), (12, "LHip"), (13, "LKnee"), (14, "LAnkle"),
                                     (15, "REye"), (16, "LEye"), (17, "REar"), (18, "LEar"), (19, "LBigToe"),
                                     (20, "LSmallToe"), (21, "LHeel"), (22, "RBigToe"), (23, "RSmallToe"), (24, "RHeel"),
                                     (25, "Background")
                                ])
    element_dict = dict([ (0, "x"), (1, "y"), (2, "z"), (3, "c") ])

    output_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/"
    output_subfolder = "take1/"
    output_folder_path = output_path + output_subfolder
    raw_output_file_prefix = "raw Tue Jan 15 14:21:2"   # to go from 14:21:20 to 14:21:29 (10 files)
    tfed_output_file_prefix = "tfed Tue Jan 15 14:21:"
    csv_folder_path = output_folder_path + "csv/"
    plots_folder_path = output_folder_path + "plots/"

    # create CSVs directory
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)

    # create plots directory
    if not os.path.exists(plots_folder_path):
        os.makedirs(plots_folder_path)

    # create CSVs
    for key, value in body_25_body_parts_dict.items():
        fp = open(csv_folder_path + value + "CoordsAndProb" + ".csv", 'w')
        fp.close()
    for key, value in body_25_body_parts_dict.items():
        fp = open(csv_folder_path + value + ".csv", 'w')
        fp.close()
    
    # create our 3d report matrix: [BodyPart][x/y/z/prob][t0,...,t9,mean,variance,skewness,kurtosis] --> 25 * 4 * 17
    part, elem, val = 25, 4, 17
    stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx = 10, 10, 11, 12, 13, 14, 15, 16
    report_matrix = [ [ [ np.nan for k in range(val) ] for j in range(elem) ] for i in range(part) ]

    # access the files of the output directory
    file_counter = 0
    for file in os.listdir(output_folder_path):
        try:
            if os.path.isfile(os.path.join(output_folder_path, file)) and raw_output_file_prefix in file:
                # read from file
                with open( os.path.join(output_folder_path, file), 'r' ) as fp:
                    for cnt, line in enumerate(fp):
                        if "Body" not in line and line.strip():     # ignore header lines and empty lines
                            body_part = re.search('kp (.*):', line).group(1)
                            coords_and_prob = re.findall(r'\d+\.\d+', line)

                            # write in the appropriate CSV
                            with open(csv_folder_path + body_part + "CoordsAndProb" + ".csv", 'a') as fp:
                                string = ""
                                coord_or_prob_idx = 0
                                for i in coords_and_prob:
                                    if string.strip():
                                        string = string + ","
                                    string = string + str(i)

                                    # fill report matrix
                                    report_matrix[ getKeysByValue(body_25_body_parts_dict, body_part)[0] ][coord_or_prob_idx][file_counter] = float(i)

                                    coord_or_prob_idx = coord_or_prob_idx + 1

                                string = string + "\n"
                                fp.write(string)
            
                file_counter = file_counter + 1
        except Exception as e:
            raise e

    # do statistical analysis
    for i in range(part):
        for j in range(elem):
            # count the non-nan values
            non_nans = (~np.isnan(report_matrix[i][j][0:stat_analysis_idx])).sum(0)
            if not non_nans:
                continue

            # source: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.describe.html
            description = stats.describe(report_matrix[i][j][0:stat_analysis_idx])
            report_matrix[i][j][nobs_idx] = description.nobs
            report_matrix[i][j][min_idx] = np.nanmin(report_matrix[i][j][0:stat_analysis_idx], axis=0)
            report_matrix[i][j][max_idx] = np.nanmax(report_matrix[i][j][0:stat_analysis_idx], axis=0)
            report_matrix[i][j][mean_idx] = description.mean
            report_matrix[i][j][variance_idx] = description.variance
            report_matrix[i][j][skewness_idx] = description.skewness
            report_matrix[i][j][kurtosis_idx] = description.kurtosis
        
            histogram(  data=report_matrix[i][j][0:stat_analysis_idx],
                        x_label=element_dict.get(j), y_label="Frequency",
                        title="Distribution of "+element_dict.get(j)+" at "+body_25_body_parts_dict.get(i),
                        directory=plots_folder_path
                    )
        
        scatterplot(    x=report_matrix[i][0][0:stat_analysis_idx], y=report_matrix[i][1][0:stat_analysis_idx], z=report_matrix[i][2][0:stat_analysis_idx],
                        x_label='X', y_label='Y', z_label='Z',
                        title="Scatterplot of X, Y, Z at "+body_25_body_parts_dict.get(i),
                        directory=plots_folder_path
                    )

    for i in range(part):
        if i < part-1:
            errorbar(  x=np.sort(np.nan_to_num(report_matrix[i][3][0:stat_analysis_idx])), y=np.sort(np.nan_to_num(report_matrix[i+1][3][0:stat_analysis_idx])),
                        x_label="Certainties", y_label="Error",
                        title="Errorplot of "+body_25_body_parts_dict.get(i)+" certainty compared to "+body_25_body_parts_dict.get(i+1)+" certainty",
                        directory=plots_folder_path,
                        x_lim_min=(report_matrix[i][3][min_idx] if report_matrix[i][3][min_idx] < report_matrix[i+1][3][min_idx] else report_matrix[i+1][3][min_idx]), x_lim_max=((report_matrix[i][3][max_idx] if report_matrix[i][3][max_idx] > report_matrix[i+1][3][max_idx] else report_matrix[i+1][3][max_idx]))
                    )

    variances, means, skewnesses, kurtoses = [], [], [], []
    for i in range(part):
        local_variances, local_means, local_skewnesses, local_kurtoses = [], [], [], []
        for j in range(elem):
            variances.append(report_matrix[i][j][variance_idx])
            means.append(report_matrix[i][j][mean_idx])
            skewnesses.append(report_matrix[i][j][skewness_idx])
            kurtoses.append(report_matrix[i][j][kurtosis_idx])
            local_variances.append(report_matrix[i][j][variance_idx])
            local_means.append(report_matrix[i][j][mean_idx])
            local_skewnesses.append(report_matrix[i][j][skewness_idx])
            local_kurtoses.append(report_matrix[i][j][kurtosis_idx])

        histogram(  data=local_variances,
                    x_label="All " + body_25_body_parts_dict.get(i) + " variances", y_label="Frequency",
                    title="Distribution of all " + body_25_body_parts_dict.get(i) + " variances",
                    directory=plots_folder_path
                )
        histogram(  data=local_means,
                    x_label="All " + body_25_body_parts_dict.get(i) + " means", y_label="Frequency",
                    title="Distribution of all " + body_25_body_parts_dict.get(i) + " means",
                    directory=plots_folder_path
                )
        histogram(  data=local_skewnesses,
                    x_label="All " + body_25_body_parts_dict.get(i) + " skewnesses", y_label="Frequency",
                    title="Distribution of all " + body_25_body_parts_dict.get(i) + " skewnesses",
                    directory=plots_folder_path
                )
        histogram(  data=local_kurtoses,
                    x_label="All " + body_25_body_parts_dict.get(i) + " kurtoses", y_label="Frequency",
                    title="Distribution of all " + body_25_body_parts_dict.get(i) + " kurtoses",
                    directory=plots_folder_path
                )

    histogram(  data=variances,
                x_label="All variances", y_label="Frequency",
                title="Distribution of all variances",
                directory=plots_folder_path
            )
    histogram(  data=means,
                x_label="All means", y_label="Frequency",
                title="Distribution of all means",
                directory=plots_folder_path
            )
    histogram(  data=skewnesses,
                x_label="All skewnesses", y_label="Frequency",
                title="Distribution of all skewnesses",
                directory=plots_folder_path
            )
    histogram(  data=kurtoses,
                x_label="All kurtoses", y_label="Frequency",
                title="Distribution of all kurtoses",
                directory=plots_folder_path
            )

    # write statistical analysis report
    for i in range(part):
       # write in the appropriate CSV
        with open(csv_folder_path + body_25_body_parts_dict.get(i) + ".csv", 'a') as fp:
            print >> fp , "elem,t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,nobs,min,max,mean,variance,skewness,kurtosis"
            for j in range(elem):
                print >> fp , element_dict.get(j) + "," + (",".join( str(e) for e in report_matrix[i][j] ))
