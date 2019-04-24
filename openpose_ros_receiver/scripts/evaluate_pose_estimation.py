#!/usr/bin/env python

# python modules
import os
import re
import math
import time
import numpy as np
from scipy import stats as stats
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
    return listOfKeys


# Re-order a list's element under a specific rule
def reorderList(lst, rule, values=None):
    new_list, new_values = [], []
    for r in rule:
        if r in lst:
            idx = lst.index(r)
            new_list.append(lst[idx])
            new_values.append(values[idx])
    return new_list, new_values


# Define a function for a 3D multi-scatterplot
def multiscatterplot3D(data, directory, names=None, x_label=None, y_label=None, z_label=None, title=None, x_lim_min=None, x_lim_max=None, y_lim_min=None, y_lim_max=None, z_lim_min=None, z_lim_max=None, borders=False, border_1_idx=None, border_2_idx=None):
    # print border_1_idx, border_2_idx
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax = fig.gca(projection='3d')
    if borders:
        if names:
            b = 0
            for d, name in zip(data, names):
                # print d, name
                x, y, z = d
                if b <= border_1_idx:
                    # print b, x, y, z, "pre_borders"
                    ax.scatter(x, y, z, marker='*', label=name, c='blue')
                elif b < border_2_idx:
                    # print b, x, y, z, "border_1_idx"
                    ax.scatter(x, y, z, marker="^", label=name, c='green')
                else:
                    # print b, x, y, z, "border_2_idx"
                    ax.scatter(x, y, z, marker='o', label=name, c='red')
                b = b + 1
        else:
            b = 0
            for d in data:
                x, y, z = d
                if b <= border_1_idx:
                    ax.scatter(x, y, z, marker='*', c='blue')
                elif b < border_2_idx:
                    ax.scatter(x, y, z, marker="^", c='green')
                else:
                    ax.scatter(x, y, z, marker='o', c='red')
                b = b + 1
    else:
        if names:
            for d, name in zip(data, names):
                x, y, z = d
                ax.scatter(x, y, z, marker='o', label=name, c='blue')
        else:
            for d in data:
                x, y, z = d
                ax.scatter(x, y, z, marker='o', c='blue')
    if x_label:
        ax.set_xlabel(x_label, fontsize=8)
    if y_label:
        ax.set_ylabel(y_label, fontsize=8)
    if z_label:
        ax.set_zlabel(z_label, fontsize=8)
    if title:
        ax.set_title(title, fontsize=10)
    if names:
        chartBox = ax.get_position()
        ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.85, chartBox.height])
        ax.legend(loc='center', bbox_to_anchor=(1.15, 0.5), shadow=True, ncol=1, borderaxespad=0, prop={'size':6})
    if x_lim_min and x_lim_max and y_lim_min and y_lim_max and z_lim_min and z_lim_max:
        # print x_lim_min, x_lim_max, y_lim_min, y_lim_max, z_lim_min, z_lim_max
        if x_lim_min != x_lim_max:
            ax.set_xlim3d(x_lim_min, x_lim_max)
        if y_lim_min != y_lim_max:
            ax.set_ylim3d(y_lim_min, y_lim_max)
        if z_lim_min != z_lim_max:
            ax.set_zlim3d(z_lim_min, z_lim_max)
    plt.savefig(directory+title+".png")
    # if len(data) > 2:
    #     plt.show(fig)
    plt.close(fig)


# Simulate a function for a boxplot to find mins and maxes of caps
def simboxplot(data):
    if not data:
        return
    fig = plt.figure()
    ax = fig.add_subplot(111)
    data_dict = ax.boxplot(data)
    temp_y_lim_min = np.nanmin([c.get_ydata()[0] for c in data_dict["caps"]])    # caps: the horizontal lines at the ends of the whiskers.
    temp_y_lim_max = np.nanmax([c.get_ydata()[0] for c in data_dict["caps"]])    # caps: the horizontal lines at the ends of the whiskers.
    plt.close(fig)

    return temp_y_lim_min, temp_y_lim_max


# Define a function for a boxplot
def boxplot(data, directory, data_label=None, y_label=None, title=None, x_tick_labels=None, y_lim_min=None, y_lim_max=None, optimize_lims=False):
    if not data:
        return
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.boxplot(data, showfliers=False)
    if data_label:
        ax.set_xlabel(data_label, fontsize=8)
    if y_label:
        ax.set_ylabel(y_label, fontsize=8)
    if x_tick_labels:
        ax.set_xticklabels(x_tick_labels, rotation=45, ha="right", fontsize=8)
    if title:
        ax.set_title(title, fontsize=10)
    if y_lim_min and y_lim_max and y_lim_min != y_lim_max:
        # to distance a bit the margins of the plot from the caps
        if optimize_lims:
            y_lim_min = y_lim_min - abs(y_lim_max - y_lim_min) / 100
            y_lim_max = y_lim_max + abs(y_lim_max - y_lim_min) / 100
        plt.ylim(y_lim_min, y_lim_max)
        # ax.yaxis.set_ticks(np.arange(y_lim_min, y_lim_max, 0.01))
    ax.margins(y=0)
    plt.savefig(directory+title+".png")
    plt.close(fig)


# MAIN FUNCTION


if __name__ == "__main__":
    # expand the color palette for the plots, once and for all
    sns.set_palette(sns.color_palette("hls", 25))

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
                                [0, 15], [15, 17], [0, 16], [16, 18], [14, 19],
                                [19, 20], [14, 21], [11, 22], [22, 23], [11, 24]
                            ]
    body_25_body_parts_LR_order_of_appearance = [ '4', '23', '3', '22', '11', '2', '10', '9', '24', '17', '15', '8', '1', '0', '16', '18', '21', '12', '13', '14', '19', '5', '6', '20', '7' ]
    body_25_upper_body_parts_LR_order_of_appearance = [ '4', '3', '2', '09', '17', '15', '8', '1', '0', '16', '18', '12', '5', '6', '7' ]
    body_25_body_parts_UL_order_of_appearance = [ '15', '16', '17', '18', '0', '2', '5', '1', '3', '6', '4', '7', '9', '12', '8', '10', '13', '11', '14', '24', '21', '23', '20', '22', '19' ]
    body_25_upper_body_parts_UL_order_of_appearance = [ '15', '16', '17', '18', '0', '2', '5', '1', '3', '6', '4', '7', '9', '12', '8' ]

    # OpenPose specific variables
    element_dict = dict([ (0, "x"), (1, "y"), (2, "z")])

    # Scenarios specific variables
    ''''''
    scenarios_dict = dict([ (0, "A-FV-ST-V-C"), (1, "A-FV-ST-V-C-right"), (2, "A-FV-ST-V-C-up"), (3, "A-FV-ST-V-C-front"), (4, "A-FV-ST-H-C"),
                            (5, "A-FV-ST-H-O"), (6, "A-FV-ST-V-O"), (7, "A-FV-MT-V-C"), (8, "A-FV-CT-V-C-up"), (9, "A-SV-ST-H-C"),
                            (10, "C-FV-ST-V-C"), (11, "C-FV-ST-V-C-right"), (12, "C-FV-ST-V-C-up"), (13, "C-FV-ST-V-C-front"), (14, "B-FV-ST-H-C"),
                            (15, "B-FV-ST-H-O"), (16, "B-FV-ST-V-O"), (17, "B-FV-MT-V-C"), (18, "B-FV-CT-V-C"), (19, "B-SV-ST-C"),
                            (20, "E-FV-ST-H-C"), (21, "E-FV-ST-H-O"), (22, "E-SV-ST-H-C"), (23, "E-SV-ST-V-C"), (24, "E-SV-ST-H-C-left"),
                            (25, "E-SV-ST-H-C-up"), (26, "E-SV-ST-H-C-front")
                        ])
    ''''''
    # scenarios_dict = dict([ (0, "A-FV-ST-V-C"), (1, "A-FV-ST-V-C-right"), (2, "A-FV-ST-V-C-up"), (3, "A-FV-ST-V-C-front"), (4, "A-FV-ST-H-C"),
    #                         (5, "A-FV-ST-H-O"), (6, "A-FV-ST-V-O"), (7, "A-FV-MT-V-C"), (8, "A-FV-CT-V-C-up"), (9, "A-SV-ST-H-C"),
    #                         (10, "C-FV-ST-V-C"), (11, "C-FV-ST-V-C-right"), (12, "C-FV-ST-V-C-up"), (13, "C-FV-ST-V-C-front"), (14, "B-FV-ST-H-C"),
    #                         (15, "B-FV-ST-H-O"), (16, "B-FV-ST-V-O"), (17, "B-FV-MT-V-C"), (18, "B-FV-CT-V-C"), (19, "B-SV-ST-C"),
    #                         (20, "E-FV-ST-H-C"), (21, "E-FV-ST-H-O"), (22, "E-SV-ST-H-C"), (23, "E-SV-ST-V-C"), (24, "E-SV-ST-H-C-left"),
    #                         (25, "E-SV-ST-H-C-up"), (26, "E-SV-ST-H-C-front"), (27, "A-FV-ST-V-C_ASTRA_VGA"), (28, "A-FV-ST-V-C_XTION_VGA"), (29, "A-FV-ST-V-C_ZED_HD720")
    #                     ])
    ''''''
    # scenarios_dict = dict([ (0, "A-FV-ST-V-C_ASTRA_VGA"), (1, "A-FV-ST-V-C_XTION_VGA"), (2, "A-FV-ST-V-C_ZED_HD720") ])
    ''''''

    complementary_scenarios_c_o_pairs = [ ["A-FV-ST-H-C", "A-FV-ST-H-O"], ["B-FV-ST-H-C", "B-FV-ST-H-O"], ["E-FV-ST-H-C", "E-FV-ST-H-O"] ]
    ''''''
    # complementary_scenarios_c_o_pairs = []
    # group_of_scenarios = ["A-FV-ST-V-C_ASTRA_VGA", "A-FV-ST-V-C_XTION_VGA", "A-FV-ST-V-C_ZED_HD720"]
    ''''''
    cube_dimensions_dict = dict([ ("l", 0.093), ("w", 0.093), ("h", 0.07) ])
    ground_truth_dict = dict([ ("A", [-0.35, -0.35, 0.09]), ("A-right", [-0.257, -0.35, 0.09]), ("A-up", [-0.35, -0.35, 0.16]), ("A-front", [-0.35, -0.275, 0.09]),
                               ("B", [-0.15, -0.185, 0.09]),
                               ("C", [0.35, -0.35, 0.09]), ("C-right", [0.443, -0.35, 0.09]), ("C-up", [0.35, -0.35, 0.16]), ("C-front", [0.35, -0.257, 0.09]),
                               ("E", [0.417, -0.035, 0.09]), ("E-left", [0.324, -0.035, 0.09]), ("E-up", [0.417, -0.035, 0.16]), ("E-front", [0.417, 0.058, 0.09])
                            ])
    ''''''
    right_wrist_ground_truth_dict = dict([ (0, [-0.35, -0.35, 0.09]), (1, [-0.257, -0.35, 0.09]), (2, [-0.35, -0.35, 0.16]), (3, [-0.35, -0.275, 0.09]), (4, [-0.35, -0.35, 0.09]),
                                           (5, [-0.35, -0.35, 0.09]), (6, [-0.35, -0.35, 0.09]), (7, [-0.35, -0.35, 0.09]), (8, [-0.35, -0.35, 0.16]), (9, [-0.35, -0.35, 0.09]),
                                           (10, [0.35, -0.35, 0.09]), (11, [0.443, -0.35, 0.09]), (12, [0.35, -0.35, 0.16]), (13, [0.35, -0.257, 0.09]), (14, [-0.15, -0.185, 0.09]),
                                           (15, [-0.15, -0.185, 0.09]), (16, [-0.15, -0.185, 0.09]), (17, [-0.15, -0.185, 0.09]), (18, [-0.15, -0.185, 0.09]), (19, [-0.15, -0.185, 0.09]),
                                           (20, [0.417, -0.035, 0.09]), (21, [0.417, -0.035, 0.09]), (22, [0.417, -0.035, 0.09]), (23, [0.417, -0.035, 0.09]), (24, [0.324, -0.035, 0.09]),
                                           (25, [0.417, -0.035, 0.16]), (26, [0.417, 0.058, 0.09])
                                        ])
    ''''''
    # right_wrist_ground_truth_dict = dict([ (0, [-0.35, -0.35, 0.09]), (1, [-0.257, -0.35, 0.09]), (2, [-0.35, -0.35, 0.16]), (3, [-0.35, -0.275, 0.09]), (4, [-0.35, -0.35, 0.09]),
    #                                        (5, [-0.35, -0.35, 0.09]), (6, [-0.35, -0.35, 0.09]), (7, [-0.35, -0.35, 0.09]), (8, [-0.35, -0.35, 0.16]), (9, [-0.35, -0.35, 0.09]),
    #                                        (10, [0.35, -0.35, 0.09]), (11, [0.443, -0.35, 0.09]), (12, [0.35, -0.35, 0.16]), (13, [0.35, -0.257, 0.09]), (14, [-0.15, -0.185, 0.09]),
    #                                        (15, [-0.15, -0.185, 0.09]), (16, [-0.15, -0.185, 0.09]), (17, [-0.15, -0.185, 0.09]), (18, [-0.15, -0.185, 0.09]), (19, [-0.15, -0.185, 0.09]),
    #                                        (20, [0.417, -0.035, 0.09]), (21, [0.417, -0.035, 0.09]), (22, [0.417, -0.035, 0.09]), (23, [0.417, -0.035, 0.09]), (24, [0.324, -0.035, 0.09]),
    #                                        (25, [0.417, -0.035, 0.16]), (26, [0.417, 0.058, 0.09]), (27, [-0.35, -0.35, 0.09]), (28, [-0.35, -0.35, 0.09]), (29, [-0.35, -0.35, 0.09])
    #                                     ])
    ''''''
    # right_wrist_ground_truth_dict = dict([ (0, [-0.35, -0.35, 0.09]), (1, [-0.35, -0.35, 0.09]), (2, [-0.35, -0.35, 0.09]) ])
    ''''''

    # File I/O specific variables
    ''''''
    scenarios_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/SCENARIOS/ZED_VGA/"
    ''''''
    # scenarios_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/SCENARIOS_expanded/"
    ''''''
    # scenarios_path = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/SCENARIOS_ASTRA_XTION_ZEDHD720/"
    ''''''
    scenarios_logs_path = scenarios_path + "logs/"
    plots_folder_path = scenarios_path + "evaluation_plots/"
    statistics_folder_path = scenarios_path + "evaluation_statistics/"
    csvs_folder_path = scenarios_path + "evaluation_csvs/"
    
    # create our 4d report matrix, e.g. for 10 log frames at 10 scenarios: [Scenario][BodyPart][x/y/z][t0,...,t9,mean,nobs,min,max,variance,skewness,kurtosis,std_dev] --> 10 * 25 * 4 * 18
    ''''''
    # max_logs = 10
    # scenarios, part, elem, val = 27, 25, 3, 18
    # stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx, std_dev_idx = 10, 10, 11, 12, 13, 14, 15, 16, 17
    ''''''
    # max_logs = 15
    # scenarios, part, elem, val = 27, 25, 3, 23
    # stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx, std_dev_idx = 15, 15, 16, 17, 18, 19, 20, 21, 22
    ''''''
    max_logs = 40
    scenarios, part, elem, val = 27, 25, 3, 48
    stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx, std_dev_idx = 40, 40, 41, 42, 43, 44, 45, 46, 47
    ''''''
    # max_logs = 10
    # scenarios, part, elem, val = 30, 25, 3, 18
    # stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx, std_dev_idx = 10, 10, 11, 12, 13, 14, 15, 16, 17
    ''''''
    # max_logs = 10
    # scenarios, part, elem, val = 3, 25, 3, 18
    # stat_analysis_idx, nobs_idx, min_idx, max_idx, mean_idx, variance_idx, skewness_idx, kurtosis_idx, std_dev_idx = 10, 10, 11, 12, 13, 14, 15, 16, 17
    ''''''

    right_wrist_idx = getKeysByValue(body_25_body_parts_dict, "RWrist")[0]
    report_matrix = [ [ [ [ np.nan for l in range(val) ] for k in range(elem) ] for j in range(part) ] for i in range(scenarios) ]

    # create plots directory
    if not os.path.exists(plots_folder_path):
        os.makedirs(plots_folder_path)

    # create statistics directory
    if not os.path.exists(statistics_folder_path):
        os.makedirs(statistics_folder_path)

    # create csvs directory
    if not os.path.exists(csvs_folder_path):
        os.makedirs(csvs_folder_path)

    # create CSVs for each scenario
    for key, value in scenarios_dict.items():
        csvs_subfolder_path = csvs_folder_path + value + "/"
        # create csvs subdirectory
        if not os.path.exists(csvs_subfolder_path):
            os.makedirs(csvs_subfolder_path)
        # create CSVs
        for k, v in body_25_body_parts_dict.items():
            fp = open(csvs_subfolder_path + v + "CoordsAndProb" + ".csv", 'w')
            fp.close()
        for k, v in body_25_body_parts_dict.items():
            fp = open(csvs_subfolder_path + v + ".csv", 'w')
            fp.close()

    # create statistics file(s)
    fp = open(statistics_folder_path + "evaluation_statistics.csv", 'w')
    fp.close()
    fp = open(statistics_folder_path + "right_wrist_statistics.csv", 'w')
    fp.close()
    fp = open(statistics_folder_path + "all_right_wrist_x.csv", 'w')
    fp.close()
    fp = open(statistics_folder_path + "all_right_wrist_y.csv", 'w')
    fp.close()
    fp = open(statistics_folder_path + "all_right_wrist_z.csv", 'w')
    fp.close()

    # access the files of the folders of the logs directory
    scenario = 0
    for file in os.listdir(scenarios_logs_path):
        try:
            if not os.path.isfile(os.path.join(scenarios_logs_path, file)):
                scenarios_logs_subfolder_path = os.path.join(scenarios_logs_path, file)
                
                file_counter = 0
                for subfile in os.listdir(scenarios_logs_subfolder_path):
                    try:
                        if os.path.isfile(os.path.join(scenarios_logs_subfolder_path, subfile)):
                            # read from file
                            with open( os.path.join(scenarios_logs_subfolder_path, subfile), 'r' ) as fp:
                                
                                for cnt, line in enumerate(fp):
                                    if "Body" not in line and line.strip():     # ignore header lines and empty lines
                                        body_part = re.search('kp (.*):', line).group(1)
                                        # source: https://stackoverflow.com/questions/678236/how-to-get-the-filename-without-the-extension-from-a-path-in-python
                                        coords_and_prob = re.findall(r'[-+]?\d+\.\d+', line)

                                        # write in the appropriate CSV
                                        with open(csvs_folder_path + os.path.splitext(os.path.basename(scenarios_logs_subfolder_path))[0] + "/" + body_part + "CoordsAndProb" + ".csv", 'a') as fp:
                                            string = ""
                                            coord_or_prob_idx = 0
                                            for i in coords_and_prob:
                                                if string.strip():
                                                    string = string + ","
                                                string = string + str(i)

                                                # fill report matrix
                                                report_matrix[ getKeysByValue(scenarios_dict, os.path.splitext(os.path.basename(scenarios_logs_subfolder_path))[0])[0] ][ getKeysByValue(body_25_body_parts_dict, body_part)[0] ][coord_or_prob_idx][file_counter] = float(i)

                                                coord_or_prob_idx = coord_or_prob_idx + 1

                                            string = string + "\n"
                                            fp.write(string)

                            file_counter = file_counter + 1

                            if file_counter == max_logs:
                                break

                    except Exception as e:
                        raise e

                scenario = scenario + 1

                if scenario == scenarios:
                    break

        except Exception as e:
            raise e

    # do statistical analysis
    occurrences_accross_frames_accross_scenarios = [ [ 0 for j in range(scenarios) ] for i in range(part) ]

    for i in range(scenarios):
        for j in range(part):
            for k in range(elem):
                # count the non-nan values
                non_nans = (~np.isnan(report_matrix[i][j][k][0:stat_analysis_idx])).sum(0)
                if not non_nans:
                    continue

                # source: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.describe.html
                description = stats.describe(np.array(report_matrix[i][j][k][0:stat_analysis_idx])[~np.isnan(report_matrix[i][j][k][0:stat_analysis_idx])])
                report_matrix[i][j][k][nobs_idx] = description.nobs
                report_matrix[i][j][k][min_idx] = np.nanmin(report_matrix[i][j][k][0:stat_analysis_idx], axis=0)
                report_matrix[i][j][k][max_idx] = np.nanmax(report_matrix[i][j][k][0:stat_analysis_idx], axis=0)
                report_matrix[i][j][k][mean_idx] = description.mean
                report_matrix[i][j][k][variance_idx] = description.variance
                report_matrix[i][j][k][skewness_idx] = description.skewness
                report_matrix[i][j][k][kurtosis_idx] = description.kurtosis
                report_matrix[i][j][k][std_dev_idx] = np.std(np.array(report_matrix[i][j][k][0:stat_analysis_idx])[~np.isnan(report_matrix[i][j][k][0:stat_analysis_idx])])

            # Count occurrences accross log frames accross scenarios
            occurrences_accross_frames_accross_scenarios[j][i] = (~np.isnan(report_matrix[i][j][ getKeysByValue(element_dict, "x")[0] ][0:stat_analysis_idx])).sum(0)


    # right wrist coords accross scenarios, with ground truth and mean value ( range(max_logs+2) )
    right_wrist_coords = [ [ 0.0 for j in range(max_logs+2) ] for i in range(scenarios) ]
    names = [ "t"+str(e) for e in range(max_logs) ]
    names.append("mean")
    names.append("GroundTruth")

    # gather the necessary right wrist coordinates
    for i in range(scenarios):
        for j in range(max_logs+2):
            if j == max_logs:
                right_wrist_coords[i][j] = [ report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][mean_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][mean_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][mean_idx] ]
            elif j == max_logs + 1:
                right_wrist_coords[i][j] = right_wrist_ground_truth_dict.get(i)
            else:
                right_wrist_coords[i][j] = [ report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] ]

    # do the plotting
    for i in range(scenarios):
        # print "all: ", len(right_wrist_coords[i][0:max_logs+2])
        # print right_wrist_coords[i][0:max_logs+2]
        # first, sanitize data
        filtered = []
        for rwc in right_wrist_coords[i]:
            x_i, y_i, z_i = rwc[0], rwc[1], rwc[2]
            if not np.isnan(x_i) and not np.isnan(y_i) and not np.isnan(z_i):
                filtered.append([x_i, y_i, z_i])
        # print "filtered: ", len(filtered)
        # print filtered
        # second, plot them
        if len(filtered) >= 3:
            multiscatterplot3D(
                data=filtered,
                directory=plots_folder_path,
                x_label="X", y_label="Y", z_label="Z",
                title=scenarios_dict.get(i) + " Right Wrist in space",
                names=names,
                borders=True, border_1_idx=len(filtered)-3, border_2_idx=len(filtered)-1,
                x_lim_min=np.min([ e[0] for e in filtered ]), x_lim_max=np.max([ e[0] for e in filtered ]),
                y_lim_min=np.min([ e[1] for e in filtered ]), y_lim_max=np.max([ e[1] for e in filtered ]),
                z_lim_min=np.min([ e[2] for e in filtered ]), z_lim_max=np.max([ e[2] for e in filtered ])
            )


    # summary of right wrist statistics accross scenarios
    right_wrist_stats, all_right_wrist_x, all_right_wrist_y, all_right_wrist_z = [], [], [], []

    for i in range(scenarios):
        right_wrist_stats.append([
            right_wrist_ground_truth_dict.get(i)[0], right_wrist_ground_truth_dict.get(i)[1], right_wrist_ground_truth_dict.get(i)[2],
            report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][mean_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][mean_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][mean_idx],
            report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][std_dev_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][std_dev_idx], report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][std_dev_idx],
            (right_wrist_ground_truth_dict.get(i)[0] - report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][mean_idx]), (right_wrist_ground_truth_dict.get(i)[1] - report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][mean_idx]), (right_wrist_ground_truth_dict.get(i)[2] - report_matrix[i][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][mean_idx])
        ])

    right_wrist_occurrences = 0

    # [Scenario][BodyPart][x/y/z][t0,...,t9,mean,nobs,min,max,variance,skewness,kurtosis,std_dev]
    for i in range(scenarios):
        if np.nan not in right_wrist_stats[i]:
            scenario_right_wrist_x, scenario_right_wrist_y, scenario_right_wrist_z = [], [], []

            for j in range(max_logs):
                x = report_matrix[i][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "x")[0] ][j]
                y = report_matrix[i][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "y")[0] ][j]
                z = report_matrix[i][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "z")[0] ][j]

                if x is not np.nan:
                    scenario_right_wrist_x.append(x)
                else:
                    scenario_right_wrist_x.append(0.0)
                if y is not np.nan:
                    scenario_right_wrist_y.append(y)
                else:
                    scenario_right_wrist_y.append(0.0)
                if z is not np.nan:
                    scenario_right_wrist_z.append(z)
                else:
                    scenario_right_wrist_z.append(0.0)

            all_right_wrist_x.append(scenario_right_wrist_x)
            all_right_wrist_y.append(scenario_right_wrist_y)
            all_right_wrist_z.append(scenario_right_wrist_z)

            right_wrist_occurrences += 1
    
    rotated_all_right_wrist_x, rotated_all_right_wrist_y, rotated_all_right_wrist_z = [], [], []

    for i in range(max_logs):
        xs, ys, zs = [], [], []
        for j in range(right_wrist_occurrences):
            xs.append( all_right_wrist_x[j][i] )
            ys.append( all_right_wrist_y[j][i] )
            zs.append( all_right_wrist_z[j][i] )

        rotated_all_right_wrist_x.append(xs)
        rotated_all_right_wrist_y.append(ys)
        rotated_all_right_wrist_z.append(zs)
    

    # complementary scenarios coordinates boxplot comparison
    # for each pair of complementary scenarios:
    for i in range(len(complementary_scenarios_c_o_pairs)):
        # take the x[], y[], z[] of each scenario
        x1, x2, y1, y2, z1, z2 = [], [], [], [], [], []
        scenario_1_idx, scenario_2_idx = getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][0])[0], getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][1])[0]
        for j in range(max_logs):
            if not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]):
                x1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] )
                x2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] )
                y1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] )
                y2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] )
                z1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] )
                z2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] )

        # Simulate a boxplot for each element to find mins and maxes of caps
        mins, maxes = [], []
        new_y_axis_min, new_y_axis_max = 0, 0
        ret = simboxplot(data=x1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=x2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        # find y axis min and max
        y_axis_min = np.nanmin(mins)
        y_axis_max = np.nanmax(maxes)
        
        # do the boxplotting
        boxplot(
            data=[x1, x2, y1, y2, z1, z2],
            directory=plots_folder_path,
            data_label="Right Wrist coordinates at Clear vs Overlapping conditions",
            title=complementary_scenarios_c_o_pairs[i][0] + " vs " + complementary_scenarios_c_o_pairs[i][1] + " right wrist coordinates",
            x_tick_labels=[ "X_c", "X_o", "Y_c", "Y_o", "Z_c", "Z_o" ],
            y_lim_min=y_axis_min, y_lim_max=y_axis_max,
            optimize_lims=True
        )


    # more boxplots
    # # [Scenario][BodyPart][x/y/z][t0,...,t9,mean,nobs,min,max,variance,skewness,kurtosis,std_dev]
    # boxplot(
    #     data=[ report_matrix[0][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "x")[0] ][:max_logs-1], report_matrix[1][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "x")[0] ][:max_logs-1], report_matrix[2][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "x")[0] ] ][:max_logs-1],
    #         directory=plots_folder_path,
    #         data_label="Right Wrist X coordinates",
    #         title="Right Wrist X coordinates for ASTRA PRO, XTION, ZED_HD720",
    #         x_tick_labels=[ "X_astra_pro", "X_xtion", "X_zed_hd720" ]
    # )
    # boxplot(
    #     data=[ report_matrix[0][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "y")[0] ][:max_logs-1], report_matrix[1][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "y")[0] ][:max_logs-1], report_matrix[2][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "y")[0] ] ][:max_logs-1],
    #         directory=plots_folder_path,
    #         data_label="Right Wrist Y coordinates",
    #         title="Right Wrist Y coordinates for ASTRA PRO, XTION, ZED_HD720",
    #         x_tick_labels=[ "Y_astra_pro", "Y_xtion", "Y_zed_hd720" ]
    # )
    # boxplot(
    #     data=[ report_matrix[0][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "z")[0] ][:max_logs-1], report_matrix[1][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "z")[0] ][:max_logs-1], report_matrix[2][ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][ getKeysByValue(element_dict, "z")[0] ] ][:max_logs-1],
    #         directory=plots_folder_path,
    #         data_label="Right Wrist Z coordinates",
    #         title="Right Wrist Z coordinates for ASTRA PRO, XTION, ZED_HD720",
    #         x_tick_labels=[ "Z_astra_pro", "Z_xtion", "Z_zed_hd720" ]
    # )


    # repeat with median normalization
    # complementary scenarios coordinates boxplot comparison
    # for each pair of complementary scenarios:
    for i in range(len(complementary_scenarios_c_o_pairs)):
        # take the x[], y[], z[] of each scenario
        x1, x2, y1, y2, z1, z2 = [], [], [], [], [], []
        scenario_1_idx, scenario_2_idx = getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][0])[0], getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][1])[0]
        for j in range(max_logs):
            if not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]):
                x1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] - report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][mean_idx] )
                x2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] - report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][mean_idx] )
                y1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] - report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][mean_idx] )
                y2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] - report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][mean_idx] )
                z1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] - report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][mean_idx] )
                z2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] - report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][mean_idx] )

        # Simulate a boxplot for each element to find mins and maxes of caps
        mins, maxes = [], []
        ret = simboxplot(data=x1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=x2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        # find y axis min and max
        y_axis_min = np.nanmin(mins)
        y_axis_max = np.nanmax(maxes)
        
        # do the boxplotting
        boxplot(
            data=[x1, x2, y1, y2, z1, z2],
            directory=plots_folder_path,
            data_label="Right Wrist coordinates at Clear vs Overlapping conditions with median normalization",
            title=complementary_scenarios_c_o_pairs[i][0] + " vs " + complementary_scenarios_c_o_pairs[i][1] + " right wrist coordinates with median normalization",
            x_tick_labels=[ "X_c", "X_o", "Y_c", "Y_o", "Z_c", "Z_o" ],
            y_lim_min=y_axis_min, y_lim_max=y_axis_max,
            optimize_lims=True
        )


    # repeat with ground truth normalization
    # complementary scenarios coordinates boxplot comparison
    # for each pair of complementary scenarios:
    for i in range(len(complementary_scenarios_c_o_pairs)):
        # take the x[], y[], z[] of each scenario
        x1, x2, y1, y2, z1, z2 = [], [], [], [], [], []
        scenario_1_idx, scenario_2_idx = getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][0])[0], getKeysByValue(scenarios_dict, complementary_scenarios_c_o_pairs[i][1])[0]
        for j in range(max_logs):
            if not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]) and not np.isnan(report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j]) and not np.isnan(report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j]):
                x1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_1_idx)[ getKeysByValue(element_dict, "x")[0] ] )
                x2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "x")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_2_idx)[ getKeysByValue(element_dict, "x")[0] ] )
                y1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_1_idx)[ getKeysByValue(element_dict, "y")[0] ] )
                y2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "y")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_2_idx)[ getKeysByValue(element_dict, "y")[0] ] )
                z1.append( report_matrix[scenario_1_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_1_idx)[ getKeysByValue(element_dict, "z")[0] ] )
                z2.append( report_matrix[scenario_2_idx][right_wrist_idx][ getKeysByValue(element_dict, "z")[0] ][j] - right_wrist_ground_truth_dict.get(scenario_2_idx)[ getKeysByValue(element_dict, "z")[0] ] )

        # Simulate a boxplot for each element to find mins and maxes of caps
        mins, maxes = [], []
        ret = simboxplot(data=x1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=x2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=y2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z1)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        ret = simboxplot(data=z2)
        if ret:
            new_y_axis_min, new_y_axis_max = ret
        mins.append(new_y_axis_min)
        maxes.append(new_y_axis_max)
        # find y axis min and max
        y_axis_min = np.nanmin(mins)
        y_axis_max = np.nanmax(maxes)
        
        # do the boxplotting
        boxplot(
            data=[x1, x2, y1, y2, z1, z2],
            directory=plots_folder_path,
            data_label="Right Wrist coordinates at Clear vs Overlapping conditions post ground truth normalization",
            title=complementary_scenarios_c_o_pairs[i][0] + " vs " + complementary_scenarios_c_o_pairs[i][1] + " right wrist coordinates post ground truth normalization",
            x_tick_labels=[ "X_c", "X_o", "Y_c", "Y_o", "Z_c", "Z_o" ],
            y_lim_min=y_axis_min, y_lim_max=y_axis_max,
            optimize_lims=True
        )


    # write statistical analysis report
    for i in range(scenarios):
        for j in range(part):
            # write in the appropriate CSV
            with open(csvs_folder_path + scenarios_dict.get(i) + "/" + body_25_body_parts_dict.get(j) + ".csv", 'a') as fp:
                print >> fp , "elem," + (",".join( "t"+str(e) for e in range(max_logs) )) + ",nobs,min,max,mean,variance,skewness,kurtosis,std_dev"
                for k in range(elem):
                    print >> fp , element_dict.get(k) + "," + (",".join( str(e) for e in report_matrix[i][j][k] ))

    # report occurrences accross frames
    with open(statistics_folder_path + "evaluation_statistics.csv", 'w') as fp:
        print >> fp , "Occurrences accross " + str(max_logs) + " log frames"
        print >> fp, "Keypoint" + "," + (",".join( str(e) for e in scenarios_dict.values() ))
        for i in range(part):
            # write in order of appearance Upper to Lower
            # print >> fp , body_25_body_parts_dict.get( int(body_25_body_parts_UL_order_of_appearance[i]) ) + "," + (",".join( str(e) for e in occurrences_accross_frames_accross_scenarios[ int(body_25_body_parts_UL_order_of_appearance[i]) ] ))
            # write in BODY_25 index order
            print >> fp , body_25_body_parts_dict.get(i) + "," + (",".join( str(e) for e in occurrences_accross_frames_accross_scenarios[i] ))

    # report right wrist statistics accross frames
    with open(statistics_folder_path + "right_wrist_statistics.csv", 'w') as fp:
        print >> fp, "Scenario,x_gt,y_gt,z_gt,x_mean,y_mean,z_mean,x_std_dev,y_std_dev,z_std_dev,x_gt_dev,y_gt_dev,z_gt_dev"
        for i in range(scenarios):
            if np.nan not in right_wrist_stats[i]:
                print >> fp , scenarios_dict.get(i) + "," + (",".join( str(e) for e in right_wrist_stats[i] ))


    # report all right wrist x coordinates accross frames
    with open(statistics_folder_path + "all_right_wrist_x.csv", 'w') as fp:
        print >> fp, ",".join( '"'+str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] )+"@"+scenarios_dict.get(i)+'"' for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        # print >> fp, ",".join( str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] ) for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        for i in range(max_logs):
            print >> fp, ",".join( str(e) for e in rotated_all_right_wrist_x[i] )
    
    # report all right wrist y coordinates accross frames
    with open(statistics_folder_path + "all_right_wrist_y.csv", 'w') as fp:
        print >> fp, ",".join( '"'+str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] )+"@"+scenarios_dict.get(i)+'"' for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        # print >> fp, ",".join( str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] ) for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        for i in range(max_logs):
            print >> fp, ",".join( str(e) for e in rotated_all_right_wrist_y[i] )
    
    # report all right wrist x coordinates accross frames
    with open(statistics_folder_path + "all_right_wrist_z.csv", 'w') as fp:
        print >> fp, ",".join( '"'+str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] )+"@"+scenarios_dict.get(i)+'"' for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        # print >> fp, ",".join( str( occurrences_accross_frames_accross_scenarios[ getKeysByValue(body_25_body_parts_dict, "RWrist")[0] ][i] ) for i in range(scenarios) if np.nan not in right_wrist_stats[i] )
        for i in range(max_logs):
            print >> fp, ",".join( str(e) for e in rotated_all_right_wrist_z[i] )
        

    print "SUCCESS!"