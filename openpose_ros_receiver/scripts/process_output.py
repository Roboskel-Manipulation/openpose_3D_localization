#!/usr/bin/env python

# python modules
import os
import re
import math
import time
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns


# Get a list of keys from dictionary which has the given value
def getKeysByValue(dictOfElements, valueToFind):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        if item[1] == valueToFind:
            listOfKeys.append(item[0])
    return  listOfKeys


# Define a function for a plot
def plot(x, y, directory, x_label=None, y_label=None, title=None, y_lim_min=None, y_lim_max=None, x_tick_labels=None):
    fig, ax = plt.subplots()
    ax.plot(x, y, marker='o', linestyle='None')
    # number of index to markers
    i = 0
    for x_i, y_i in zip(x, y):
        ax.text(x_i, y_i, str(i), fontsize=10)
        i = i+1
    if x_label:
        ax.set_xlabel(x_label)
    if y_label:
        ax.set_ylabel(y_label)
    if x_tick_labels:
        ax.set_xticklabels(x_tick_labels)
    if title:
        ax.set_title(title)
    if y_lim_min and y_lim_max:
        plt.ylim(y_lim_min, y_lim_max)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# Define a function for multiple plots
def multiplot(x, y_data, directory, y_names=None, x_label=None, y_label=None, title=None, y_lim_min=None, y_lim_max=None, x_tick_labels=None):
    # expand the color palette for the plots
    sns.set_palette(sns.color_palette("hls", len(y_data)))
    if y_names:
        fig, (ax, lax) = plt.subplots(ncols=2, gridspec_kw={"width_ratios":[5,1]})
        for i in range(len(y_data)):
            ax.plot(x, y_data[i], label=y_names[i], linestyle='None', marker='o')
            # number of index to markers
            j = 0
            for x_j, y_j in zip(x, y_data[i]):
                ax.text(x_j, y_j, str(j), fontsize=10)
                j = j+1
    else:
        fig, ax = plt.subplots()
        for y in y_data:
            ax.plot(x, y, linestyle='--', marker='o')
            # number of index to markers
            i = 0
            for x_i, y_i in zip(x, y):
                ax.text(x_i, y_i, str(i), fontsize=10)
                i = i+1
    if x_label:
        ax.set_xlabel(x_label)
    if y_label:
        ax.set_ylabel(y_label)
    if x_tick_labels:
        ax.set_xticklabels(x_tick_labels)
    if title:
        ax.set_title(title)
    if y_names:
        h,l = ax.get_legend_handles_labels()
        lax.legend(h, l, borderaxespad=0, prop={'size': 10})
        lax.axis("off")
        plt.tight_layout()
    if y_lim_min and y_lim_max:
        plt.ylim(y_lim_min, y_lim_max)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# Define a function for a 3D scatterplot
def scatterplot(x, y, z, directory, x_label=None, y_label=None, z_label=None, title=None, x_lim_min=None, x_lim_max=None, y_lim_min=None, y_lim_max=None, z_lim_min=None, z_lim_max=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='r', marker='o')
    # number of index to markers
    i = 0
    for x_i, y_i, z_i in zip(np.array(x)[~np.isnan(np.array(x))], np.array(y)[~np.isnan(np.array(y))], np.array(z)[~np.isnan(np.array(z))]):
        ax.text(x_i, y_i, z_i, str(i), fontsize=10)
        i = i+1
    if x_label:
        ax.set_xlabel(x_label)
    if y_label:
        ax.set_ylabel(y_label)
    if z_label:
        ax.set_zlabel(z_label)
    if title:
        ax.set_title(title)
    if x_lim_min and x_lim_max and y_lim_min and y_lim_max and z_lim_min and z_lim_max:
        if x_lim_min != x_lim_max:
            ax.set_xlim3d(x_lim_min, x_lim_max)
        if y_lim_min != y_lim_max:
            ax.set_ylim3d(y_lim_min, y_lim_max)
        if z_lim_min != z_lim_max:
            ax.set_zlim3d(z_lim_min, z_lim_max)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# Define a function for a 3D multi-scatterplot
def multiscatterplot(data, directory, names=None, x_label=None, y_label=None, z_label=None, title=None, x_lim_min=None, x_lim_max=None, y_lim_min=None, y_lim_max=None, z_lim_min=None, z_lim_max=None):
    colors = ['red', 'green', 'silver', 'rosybrown', 'firebrick',
            'grey', 'darksalmon', 'sienna', 'sandybrown', 'darkkhaki',
            'palegreen', 'lightseagreen', 'darkcyan', 'paleturquoise', 'deepskyblue',
            'royalblue', 'navy', 'lightcoral', 'brown', 'y',
            'limegreen', 'teal', 'steelblue', 'darkmagenta', 'peru']
    # print len(data)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax = fig.gca(projection='3d')
    if names:
        j = 0
        for d, name in zip(data, names):
            x, y, z = d
            ax.scatter(x, y, z, marker='o', label=name, c=colors[j])
            j = j + 1
            # # number of index to markers
            # i = 0
            # for x_i, y_i, z_i in zip(x, y, z):
            #     ax.text(x_i, y_i, z_i, str(i), fontsize=10)
            #     i = i+1
    else:
        for d in data:
            x, y, z = d
            ax.scatter(x, y, z, marker='o')
            # # number of index to markers
            # i = 0
            # for x_i, y_i, z_i in zip(x, y, z):
            #     ax.text(x_i, y_i, z_i, str(i), fontsize=10)
            #     i = i+1
    if x_label:
        ax.set_xlabel(x_label)
    if y_label:
        ax.set_ylabel(y_label)
    if z_label:
        ax.set_zlabel(z_label)
    if title:
        ax.set_title(title)
    if names:
        chartBox = ax.get_position()
        ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.85, chartBox.height])
        ax.legend(loc='center', bbox_to_anchor=(1.15, 0.5), shadow=True, ncol=1)
        # plt.legend(loc=0)
    if x_lim_min and x_lim_max and y_lim_min and y_lim_max and z_lim_min and z_lim_max:
        if x_lim_min != x_lim_max:
            ax.set_xlim3d(x_lim_min, x_lim_max)
        if y_lim_min != y_lim_max:
            ax.set_ylim3d(y_lim_min, y_lim_max)
        if z_lim_min != z_lim_max:
            ax.set_zlim3d(z_lim_min, z_lim_max)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# Define a function for a histogram
def histogram(data, directory, x_label=None, y_label=None, title=None):
    fig, ax = plt.subplots()
    ax.hist(np.array(data)[~np.isnan(np.array(data))], color = '#539caf')
    if x_label:
        ax.set_xlabel(x_label)
    if y_label:
        ax.set_ylabel(y_label)
    if title:
        ax.set_title(title)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# Define a function for a boxplot
def boxplot(data, directory, data_label=None, y_label=None, title=None, x_tick_labels=None):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.boxplot(data)
    if data_label:
        ax.set_xlabel(data_label)
    if y_label:
        ax.set_ylabel(y_label)
    if x_tick_labels:
        ax.set_xticklabels(x_tick_labels)
    if title:
        ax.set_title(title)
    plt.savefig(directory+title+".png")
    plt.close(fig)


if __name__ == "__main__":

    # Body 25 human pose model specific variables
    body_25_body_parts_dict = dict([ (0,  "Nose"), (1,  "Neck"), (2,  "RShoulder"), (3,  "RElbow"), (4,  "RWrist"),
                                     (5,  "LShoulder"), (6,  "LElbow"), (7,  "LWrist"), (8,  "MidHip"), (9,  "RHip"),
                                     (10, "RKnee"), (11, "RAnkle"), (12, "LHip"), (13, "LKnee"), (14, "LAnkle"),
                                     (15, "REye"), (16, "LEye"), (17, "REar"), (18, "LEar"), (19, "LBigToe"),
                                     (20, "LSmallToe"), (21, "LHeel"), (22, "RBigToe"), (23, "RSmallToe"), (24, "RHeel"),
                                     (25, "Background")
                                ])
    body_25_body_part_pairs = [ [1, 8], [1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7],
                                [8, 9], [9, 10], [10, 11], [8, 12], [12, 13], [13, 14], [1, 0],
                                [0, 15], [15, 17], [0, 16], [16, 18], [2, 17], [5, 18], [14, 19],
                                [19, 20], [14, 21], [11, 22], [22, 23], [11, 24]
                            ]
    # OpenPose specific variables
    element_dict = dict([ (0, "x"), (1, "y"), (2, "z"), (3, "certainty") ])

    # File I/O specific variables
    output_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/"
    output_subfolder = "take1/"
    # output_subfolder = "take4/"
    output_folder_path = output_path + output_subfolder
    raw_output_file_prefix = "raw Tue Jan 15 14:21:2"   # to go from 14:21:20 to 14:21:29 (10 files -- 10 log frames)
    tfed_output_file_prefix = "tfed Tue Jan 15 14:21:"
    # raw_output_file_prefix = "raw Fri Jan 15 15:18:0"   # to go from 15:18:00 to 15:18:20 (20 files -- 20 log frames)
    # tfed_output_file_prefix = "tfed Fri Jan 15 15:18:"
    csv_folder_path = output_folder_path + "csv/"
    plots_folder_path = output_folder_path + "plots/"
    statistics_folder_path = output_folder_path + "statistics/"

    # create CSVs directory
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)

    # create plots directory
    if not os.path.exists(plots_folder_path):
        os.makedirs(plots_folder_path)
    
    # create statistics directory
    if not os.path.exists(statistics_folder_path):
        os.makedirs(statistics_folder_path)

    # create CSVs
    for key, value in body_25_body_parts_dict.items():
        fp = open(csv_folder_path + value + "CoordsAndProb" + ".csv", 'w')
        fp.close()
    for key, value in body_25_body_parts_dict.items():
        fp = open(csv_folder_path + value + ".csv", 'w')
        fp.close()
    
    # create statistics file(s)
    fp = open(statistics_folder_path + "Statistics.txt", 'w')
    fp.close()
    
    # create our 3d report matrix for 10 log frames: [BodyPart][x/y/z/prob][t0,...,t9,mean,variance,skewness,kurtosis] --> 25 * 4 * 17
    part, elem, val = 25, 4, 17
    stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx = 10, 10, 11, 12, 13, 14, 15, 16
    report_matrix = [ [ [ np.nan for k in range(val) ] for j in range(elem) ] for i in range(part) ]
    # # create our 3d report matrix for 20 log frames: [BodyPart][x/y/z/prob][t0,...,t19,mean,variance,skewness,kurtosis] --> 25 * 4 * 27
    # part, elem, val = 25, 4, 27
    # stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx = 20, 20, 21, 22, 23, 24, 25, 26
    # report_matrix = [ [ [ np.nan for k in range(val) ] for j in range(elem) ] for i in range(part) ]

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
    occurences_accross_frames = [ 0 for i in range(part) ]
    certainty_accross_frames = [ [ 0.0 for k in range(stat_analysis_idx) ] for i in range(part) ]

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
        
            # Plot a histogram of the values of a certain element (x,y,z,certainty) of a certain body part's keypoints
            histogram(  data=report_matrix[i][j][0:stat_analysis_idx],
                        x_label=element_dict.get(j), y_label="Frequency",
                        title="Distribution of "+element_dict.get(j)+" at "+body_25_body_parts_dict.get(i),
                        directory=plots_folder_path
                    )

            # Plot a boxplot of the values of a certain element (x,y,z,certainty) of a certain body part's keypoints
            # first, sanitize data
            data = np.array(report_matrix[i][j][0:stat_analysis_idx])[~np.isnan(np.array(report_matrix[i][j][0:stat_analysis_idx]))]
            # second, plot them
            boxplot(    data=data,
                        data_label=element_dict.get(j),
                        title="Boxplot of "+element_dict.get(j)+" at "+body_25_body_parts_dict.get(i),
                        directory=plots_folder_path
                    )
        

        # Do a scatterplot of a certain body part's keypoints detected in space
        if (~np.isnan(report_matrix[i][0][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[i][1][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[i][2][0:stat_analysis_idx])).sum(0):
            scatterplot(    x=report_matrix[i][0][0:stat_analysis_idx], y=report_matrix[i][1][0:stat_analysis_idx], z=report_matrix[i][2][0:stat_analysis_idx],
                            x_label='X', y_label='Y', z_label='Z',
                            title="Scatterplot of X, Y, Z at "+body_25_body_parts_dict.get(i),
                            directory=plots_folder_path,
                            x_lim_min=np.nanmin(report_matrix[i][0][0:stat_analysis_idx]), x_lim_max=np.nanmax(report_matrix[i][0][0:stat_analysis_idx]),
                            y_lim_min=np.nanmin(report_matrix[i][1][0:stat_analysis_idx]), y_lim_max=np.nanmax(report_matrix[i][1][0:stat_analysis_idx]),
                            z_lim_min=np.nanmin(report_matrix[i][2][0:stat_analysis_idx]), z_lim_max=np.nanmax(report_matrix[i][2][0:stat_analysis_idx])
                        )

        # Do a boxplot for a certain body part's keypoints elements
        # first, sanitize data
        x = np.array(report_matrix[i][0][0:stat_analysis_idx])[~np.isnan(np.array(report_matrix[i][0][0:stat_analysis_idx]))]
        y = np.array(report_matrix[i][1][0:stat_analysis_idx])[~np.isnan(np.array(report_matrix[i][1][0:stat_analysis_idx]))]
        z = np.array(report_matrix[i][2][0:stat_analysis_idx])[~np.isnan(np.array(report_matrix[i][2][0:stat_analysis_idx]))]
        # second, plot them
        boxplot(    data=[x, y, z],
                    data_label=body_25_body_parts_dict.get(i),
                    title="Boxplot of X, Y, Z at "+body_25_body_parts_dict.get(i),
                    directory=plots_folder_path,
                    x_tick_labels=["X", "Y", "Z"]
                )

        # Count occurences accross log frames
        occurences_accross_frames[i] = (~np.isnan(report_matrix[i][ getKeysByValue(element_dict, "certainty")[0] ][0:stat_analysis_idx])).sum(0)

        # Log certainty accross frames
        for k in range(stat_analysis_idx):
            if ~np.isnan(report_matrix[i][ getKeysByValue(element_dict, "certainty")[0] ][k]):
                certainty_accross_frames[i][k] = report_matrix[i][ getKeysByValue(element_dict, "certainty")[0] ][k]


    variances, means, skewnesses, kurtoses = [], [], [], []
    x_variances, y_variances, z_variances, certainty_variances = [], [], [], []
    x_means, y_means, z_means, certainty_means = [], [], [], []
    x_skewnesses, y_skewnesses, z_skewnesses, certainty_skewnesses = [], [], [], []
    x_kurtoses, y_kurtoses, z_kurtoses, certainty_kurtoses = [], [], [], []
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


        # Plot histograms of the values of all variances, means, skewnesses and kurtoses of a certain body part
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


        # collect x,y,z,certainty means, variances, skewnesses and kurtoses
        x_variances.append(report_matrix[i][getKeysByValue(element_dict, "x")[0]][variance_idx])
        y_variances.append(report_matrix[i][getKeysByValue(element_dict, "y")[0]][variance_idx])
        z_variances.append(report_matrix[i][getKeysByValue(element_dict, "z")[0]][variance_idx])
        certainty_variances.append(report_matrix[i][getKeysByValue(element_dict, "certainty")[0]][variance_idx])
        x_means.append(report_matrix[i][getKeysByValue(element_dict, "x")[0]][mean_idx])
        y_means.append(report_matrix[i][getKeysByValue(element_dict, "y")[0]][mean_idx])
        z_means.append(report_matrix[i][getKeysByValue(element_dict, "z")[0]][mean_idx])
        certainty_means.append(report_matrix[i][getKeysByValue(element_dict, "certainty")[0]][mean_idx])
        x_skewnesses.append(report_matrix[i][getKeysByValue(element_dict, "x")[0]][skewness_idx])
        y_skewnesses.append(report_matrix[i][getKeysByValue(element_dict, "y")[0]][skewness_idx])
        z_skewnesses.append(report_matrix[i][getKeysByValue(element_dict, "z")[0]][skewness_idx])
        certainty_skewnesses.append(report_matrix[i][getKeysByValue(element_dict, "certainty")[0]][skewness_idx])
        x_kurtoses.append(report_matrix[i][getKeysByValue(element_dict, "x")[0]][kurtosis_idx])
        y_kurtoses.append(report_matrix[i][getKeysByValue(element_dict, "y")[0]][kurtosis_idx])
        z_kurtoses.append(report_matrix[i][getKeysByValue(element_dict, "z")[0]][kurtosis_idx])
        certainty_kurtoses.append(report_matrix[i][getKeysByValue(element_dict, "certainty")[0]][kurtosis_idx])


    # Plot histograms of the values of all variances, means, skewnesses and kurtoses
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


    # Plot histograms of the values of the variances, means, skewnesses and kurtoses of each keypoint elements
    histogram(  data=x_variances,
                x_label="All x variances", y_label="Frequency",
                title="Distribution of all x variances",
                directory=plots_folder_path
            )
    histogram(  data=x_means,
                x_label="All x means", y_label="Frequency",
                title="Distribution of all x means",
                directory=plots_folder_path
            )
    histogram(  data=x_skewnesses,
                x_label="All x skewnesses", y_label="Frequency",
                title="Distribution of all x skewnesses",
                directory=plots_folder_path
            )
    histogram(  data=x_kurtoses,
                x_label="All x kurtoses", y_label="Frequency",
                title="Distribution of all x kurtoses",
                directory=plots_folder_path
            )
    histogram(  data=y_variances,
                x_label="All y variances", y_label="Frequency",
                title="Distribution of all y variances",
                directory=plots_folder_path
            )
    histogram(  data=y_means,
                x_label="All y means", y_label="Frequency",
                title="Distribution of all y means",
                directory=plots_folder_path
            )
    histogram(  data=y_skewnesses,
                x_label="All y skewnesses", y_label="Frequency",
                title="Distribution of all y skewnesses",
                directory=plots_folder_path
            )
    histogram(  data=y_kurtoses,
                x_label="All y kurtoses", y_label="Frequency",
                title="Distribution of all y kurtoses",
                directory=plots_folder_path
            )
    histogram(  data=z_variances,
                x_label="All z variances", y_label="Frequency",
                title="Distribution of all z variances",
                directory=plots_folder_path
            )
    histogram(  data=z_means,
                x_label="All z means", y_label="Frequency",
                title="Distribution of all z means",
                directory=plots_folder_path
            )
    histogram(  data=z_skewnesses,
                x_label="All z skewnesses", y_label="Frequency",
                title="Distribution of all z skewnesses",
                directory=plots_folder_path
            )
    histogram(  data=z_kurtoses,
                x_label="All z kurtoses", y_label="Frequency",
                title="Distribution of all z kurtoses",
                directory=plots_folder_path
            )
    histogram(  data=certainty_variances,
                x_label="All certainty variances", y_label="Frequency",
                title="Distribution of all certainty variances",
                directory=plots_folder_path
            )
    histogram(  data=certainty_means,
                x_label="All certainty means", y_label="Frequency",
                title="Distribution of all certainty means",
                directory=plots_folder_path
            )
    histogram(  data=certainty_skewnesses,
                x_label="All certainty skewnesses", y_label="Frequency",
                title="Distribution of all certainty skewnesses",
                directory=plots_folder_path
            )
    histogram(  data=certainty_kurtoses,
                x_label="All certainty kurtoses", y_label="Frequency",
                title="Distribution of all certainty kurtoses",
                directory=plots_folder_path
            )


    # Collect the elements of each body part
    x_coll, y_coll, z_coll, certainty_coll = [], [], [], []
    for i in range(part):
        x_coll.append(report_matrix[i][ getKeysByValue(element_dict, "x")[0] ][0:stat_analysis_idx])
        y_coll.append(report_matrix[i][ getKeysByValue(element_dict, "y")[0] ][0:stat_analysis_idx])
        z_coll.append(report_matrix[i][ getKeysByValue(element_dict, "z")[0] ][0:stat_analysis_idx])
        certainty_coll.append(report_matrix[i][ getKeysByValue(element_dict, "certainty")[0] ][0:stat_analysis_idx])


    # Sanitize collected data
    for i in range(len(x_coll)):
        x_coll[i] = np.array(x_coll[i])[~np.isnan(x_coll[i])]
        y_coll[i] = np.array(y_coll[i])[~np.isnan(y_coll[i])]
        z_coll[i] = np.array(z_coll[i])[~np.isnan(z_coll[i])]
        certainty_coll[i] = np.array(certainty_coll[i])[~np.isnan(certainty_coll[i])]

    # Do a boxplot for each body parts' element
    boxplot(    data=x_coll,
                data_label="BODY_25 human pose model body parts",
                title="Boxplot of x values for all BODY_25 human pose model body parts",
                directory=plots_folder_path,
                x_tick_labels=[str(i) for i in body_25_body_parts_dict]
            )
    boxplot(    data=y_coll,
                data_label="BODY_25 human pose model body parts",
                title="Boxplot of y value for all BODY_25 human pose model body parts",
                directory=plots_folder_path,
                x_tick_labels=[str(i) for i in body_25_body_parts_dict]
            )
    boxplot(    data=z_coll,
                data_label="BODY_25 human pose model body parts",
                title="Boxplot of z value for all BODY_25 human pose model body parts",
                directory=plots_folder_path,
                x_tick_labels=[str(i) for i in body_25_body_parts_dict]
            )
    boxplot(    data=certainty_coll,
                data_label="BODY_25 human pose model body parts",
                title="Boxplot of certainty value for all BODY_25 human pose model body parts",
                directory=plots_folder_path,
                x_tick_labels=[str(i) for i in body_25_body_parts_dict]
            )


    # Plot certainty accross frames
    # first, for each body part individually
    for i in range(part):
        plot(   x=[ j for j in range(stat_analysis_idx) ],
                y=certainty_accross_frames[i],
                x_label="Frame",
                y_label=body_25_body_parts_dict.get(i) + " certainty",
                title="Plot of " + body_25_body_parts_dict.get(i) + " certainty accross frames",
                directory=plots_folder_path,
                x_tick_labels=[str(j) for j in range(stat_analysis_idx)]
            )
    # second, for all body part pairs
    for pair in body_25_body_part_pairs:
        plot(   x=certainty_accross_frames[pair[0]],
                y=certainty_accross_frames[pair[1]],
                x_label=body_25_body_parts_dict.get(pair[0]) + " certainty",
                y_label=body_25_body_parts_dict.get(pair[1]) + " certainty",
                title="Plot of " + body_25_body_parts_dict.get(pair[0]) + " and " + body_25_body_parts_dict.get(pair[1]) + " certainty accross frames",
                directory=plots_folder_path
            )
    # third, for all body parts collectivelly
    multiplot(  x=[ i for i in range(stat_analysis_idx) ],
                y_data=certainty_accross_frames,
                y_names=[ body_25_body_parts_dict.get(i) for i in range(part) ],
                y_lim_min=0.0,
                y_lim_max=1.0,
                x_label="Frame",
                y_label="Body parts certainty",
                title="Plot of body parts certainty accross frames",
                directory=plots_folder_path,
                x_tick_labels=[str(j) for j in range(stat_analysis_idx)]
            )


    # Do a scatterplot for all body part pairs detected in space
    for pair in body_25_body_part_pairs:
        if (~np.isnan(report_matrix[ pair[0] ][0][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[ pair[0] ][1][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[ pair[0] ][2][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[ pair[1] ][0][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[ pair[1] ][1][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[ pair[1] ][2][0:stat_analysis_idx])).sum(0):
            x1 = np.array(report_matrix[ pair[0] ][0][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[0] ][0][0:stat_analysis_idx])]
            y1 = np.array(report_matrix[ pair[0] ][1][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[0] ][1][0:stat_analysis_idx])]
            z1 = np.array(report_matrix[ pair[0] ][2][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[0] ][2][0:stat_analysis_idx])]
            x2 = np.array(report_matrix[ pair[1] ][0][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[1] ][0][0:stat_analysis_idx])]
            y2 = np.array(report_matrix[ pair[1] ][1][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[1] ][1][0:stat_analysis_idx])]
            z2 = np.array(report_matrix[ pair[1] ][2][0:stat_analysis_idx])[~np.isnan(report_matrix[ pair[1] ][2][0:stat_analysis_idx])]
            multiscatterplot(   data=[[x1, y1, z1], [x2, y2, z2]],
                                x_label='X', y_label='Y', z_label='Z',
                                title="Scatterplot of X, Y, Z at "+body_25_body_parts_dict.get(pair[0])+" and "+body_25_body_parts_dict.get(pair[1])+" pair",
                                directory=plots_folder_path,
                                x_lim_min=np.min([np.min(x1), np.min(x2)]), x_lim_max=np.max([np.max(x1), np.max(x2)]),
                                y_lim_min=np.min([np.min(y1), np.min(y2)]), y_lim_max=np.max([np.max(y1), np.max(y2)]),
                                z_lim_min=np.min([np.min(z1), np.min(z2)]), z_lim_max=np.max([np.max(z1), np.max(z2)]),
                                names=[body_25_body_parts_dict.get(pair[0]), body_25_body_parts_dict.get(pair[1])]
                            )


    # Do a scatterplot for all body parts detected in space
    data, names, x_mins, x_maxes, y_mins, y_maxes, z_mins, z_maxes = [], [], [], [], [], [], [], []
    for i in range(part):
        if (~np.isnan(report_matrix[i][0][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[i][1][0:stat_analysis_idx])).sum(0) and (~np.isnan(report_matrix[i][2][0:stat_analysis_idx])).sum(0):
            x = np.array(report_matrix[i][0][0:stat_analysis_idx])[~np.isnan(report_matrix[i][0][0:stat_analysis_idx])]
            y = np.array(report_matrix[i][1][0:stat_analysis_idx])[~np.isnan(report_matrix[i][1][0:stat_analysis_idx])]
            z = np.array(report_matrix[i][2][0:stat_analysis_idx])[~np.isnan(report_matrix[i][2][0:stat_analysis_idx])]
            x_mins.append(np.min(x))
            x_maxes.append(np.max(x))
            y_mins.append(np.min(y))
            y_maxes.append(np.max(y))
            z_mins.append(np.min(z))
            z_maxes.append(np.max(z))
            data.append([x, y, z])
            names.append(body_25_body_parts_dict.get(i))

    multiscatterplot(   data=data,
                        x_label='X', y_label='Y', z_label='Z',
                        title="Scatterplot of X, Y, Z for all body parts detected in space",
                        directory=plots_folder_path,
                        x_lim_min=np.min(x_mins), x_lim_max=np.max(x_maxes),
                        y_lim_min=np.min(y_mins), y_lim_max=np.max(y_maxes),
                        z_lim_min=np.min(z_mins), z_lim_max=np.max(z_maxes),
                        names=names
                    )

    # write statistical analysis report
    for i in range(part):
       # write in the appropriate CSV
        with open(csv_folder_path + body_25_body_parts_dict.get(i) + ".csv", 'a') as fp:
            print >> fp , "elem,t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,nobs,min,max,mean,variance,skewness,kurtosis"
            for j in range(elem):
                print >> fp , element_dict.get(j) + "," + (",".join( str(e) for e in report_matrix[i][j] ))


    # report occurences accross frames
    with open(statistics_folder_path + "Statistics.txt", 'w') as fp:
        print >> fp , "BODY_25 human pose model body part,Occurences accross " + str(stat_analysis_idx) + " log frames"
        for i in range(part):
            print >> fp , body_25_body_parts_dict.get(i) + "," + str(occurences_accross_frames[i])