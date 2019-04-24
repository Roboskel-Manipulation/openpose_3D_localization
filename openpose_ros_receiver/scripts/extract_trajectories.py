#!/usr/bin/env python

# python modules
import os
import numpy as np
import json
import pandas as pd
import matplotlib.pyplot as plt
from pandas.tools.plotting import autocorrelation_plot
from pandas import Series
from pandas import DataFrame
from pandas import TimeGrouper


# source: https://stackoverflow.com/questions/5389507/iterating-over-every-two-elements-in-a-list
def grouped(iterable, n):
    "s -> (s0,s1,s2,...sn-1), (sn,sn+1,sn+2,...s2n-1), (s2n,s2n+1,s2n+2,...s3n-1), ..."
    return zip(*[iter(iterable)]*n)


# Get a list of keys from dictionary which has the given value
def getKeysByValue(dictOfElements, valueToFind):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        if item[1] == valueToFind:
            listOfKeys.append(item[0])
    return listOfKeys


# Define a function for a lineplot
def lineplot(data, path, x_label=None, y_label=None, title=None):
    data.plot()
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        plt.title(title, fontsize=10)
    plt.savefig(path)
    plt.close()


# Define a function for a histogram
def histogram(data, path, x_label=None, y_label=None, title=None):
    data.hist()
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        plt.title(title, fontsize=10)
    plt.savefig(path)
    plt.close()


# Define a function for a density plot
def density_plot(data, path, x_label=None, y_label=None, title=None):
    data.plot(kind='kde')
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        plt.title(title, fontsize=10)
    plt.savefig(path)
    plt.close()


# Define a function for a plot
def plot(x_data, y_data, path, x_label=None, y_label=None, title=None):
    plt.plot(x_data, y_data, linestyle='--', marker='o')
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        plt.title(title, fontsize=10)
    plt.savefig(path)
    plt.close()


# Define a function for multiple plots
def multiplot(x_data, y_data, path, data_names=None, x_label=None, y_label=None, title=None):
    assert( len(x_data) == len(y_data) )
    data = [ [x_data[i], y_data[i]] for i in range(len(x_data)) ]

    if data_names:
        fig, (ax, lax) = plt.subplots(ncols=2, gridspec_kw={"width_ratios":[6,1]})
        i = 0
        for d in data:
            x, y = d[0], d[1]
            ax.plot(x, y, label=data_names[i], linestyle='--', marker='o')
            i += 1
    else:
        fig, ax = plt.subplots()
        for d in data:
            x, y = d[0], d[1]
            ax.plot(x, y, linestyle='--', marker='o')
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        ax.set_title(title, fontsize=10)
    if data_names:
        h,l = ax.get_legend_handles_labels()
        lax.legend(h, l, borderaxespad=0, prop={'size': 8})
        lax.axis("off")
        plt.tight_layout()
    plt.savefig(path)
    plt.close()


# Define a function for a boxplot
def boxplot(data, path, x_label=None, y_label=None, title=None):
    data.boxplot()
    if x_label:
        plt.xlabel(x_label, fontsize=8)
    if y_label:
        plt.ylabel(y_label, fontsize=8)
    if title:
        plt.title(title, fontsize=10)
    plt.savefig(path)
    plt.close()


# MAIN FUNCTION


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
                                [0, 15], [15, 17], [0, 16], [16, 18], [14, 19],
                                [19, 20], [14, 21], [11, 22], [22, 23], [11, 24]
                            ]
    body_25_body_part_pairs_dict = dict([   (1, [0, 2, 5, 8]), (2, [3]), (3, [4]), (5, [6]), (6, [7]),
                                            (8, [9, 12]), (9, [10]), (10, [11]), (12, [13]), (13, [14]),
                                            (0, [15, 16]), (15, [17]), (16, [18]), (14, [19, 21]),
                                            (19, [20]), (11, [22, 24]), (22, [23])
                                        ])

    # OpenPose specific variables
    element_dict = dict([ (0, "x"), (1, "y")])

    # File I/O specific variables
    json_dir = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/SCENARIOS/ZED_VGA/json/samples/"
    folder_name = "sample1"
    json_path = json_dir + folder_name
    trajectories_dir = "trajectories/"
    trajectories_path = json_dir + trajectories_dir
    trajectories_source = "people"
    trajectories_entry = "pose_keypoints_2d"
    trajectories_csvs_dir = "csvs/"
    trajectories_csvs_path = trajectories_path + trajectories_csvs_dir
    trajectories_plots_dir = "plots/"
    trajectories_plots_path = trajectories_path + trajectories_plots_dir
    trajectories_plots_trajectories_dir = "trajectories/"
    trajectories_plots_trajectories_path = trajectories_plots_path + trajectories_plots_trajectories_dir
    trajectories_plots_lines_dir = "lines/"
    trajectories_plots_lines_path = trajectories_plots_path + trajectories_plots_lines_dir
    trajectories_plots_histograms_dir = "histograms/"
    trajectories_plots_histograms_path = trajectories_plots_path + trajectories_plots_histograms_dir
    trajectories_plots_boxplots_dir = "boxplots/"
    trajectories_plots_boxplots_path = trajectories_plots_path + trajectories_plots_boxplots_dir
    trajectories_plots_autocorrelation_dir = "autocorrelation/"
    trajectories_plots_autocorrelation_path = trajectories_plots_path + trajectories_plots_autocorrelation_dir

    # Task specific variables
    trajectories_idx = 0
    # we expect [..., x_i, y_i, prob_i, ...] grouping for our 2d human body pose list
    grouping_factor = 3
    # create our 3d report matrix, e.g. for 10 log frames: [BodyPart][t0,...,t9][x/y] --> 25 * 10 * 2
    max_logs = 60
    part, val, elem = 25, max_logs, 2
    report_matrix = [ [ [ np.nan for k in range(elem) ] for j in range(val) ] for i in range(part) ]

    # create trajectories directory
    if not os.path.exists(trajectories_path):
        os.makedirs(trajectories_path)

    # create trajectories csvs directory
    if not os.path.exists(trajectories_csvs_path):
        os.makedirs(trajectories_csvs_path)

    # create trajectories plots directory
    if not os.path.exists(trajectories_plots_path):
        os.makedirs(trajectories_plots_path)

    # create trajectories plots trajectories directory
    if not os.path.exists(trajectories_plots_trajectories_path):
        os.makedirs(trajectories_plots_trajectories_path)

    # create trajectories plots lines directory
    if not os.path.exists(trajectories_plots_lines_path):
        os.makedirs(trajectories_plots_lines_path)

    # create trajectories plots histograms directory
    if not os.path.exists(trajectories_plots_histograms_path):
        os.makedirs(trajectories_plots_histograms_path)

    # create trajectories plots boxplots directory
    if not os.path.exists(trajectories_plots_boxplots_path):
        os.makedirs(trajectories_plots_boxplots_path)

    # create trajectories plots autocorrelation directory
    if not os.path.exists(trajectories_plots_autocorrelation_path):
        os.makedirs(trajectories_plots_autocorrelation_path)

    # create CSVs
    fp = open(trajectories_csvs_path + "all_keypoints_timeseries" + ".csv", 'w')
    fp.close()
    for _, value in body_25_body_parts_dict.items():
        fp = open(trajectories_csvs_path + value + "_x" + ".csv", 'w')
        fp.close()
        fp = open(trajectories_csvs_path + value + "_y" + ".csv", 'w')
        fp.close()


    # access the files of the folders of the logs directory
    log = 0
    for file in os.listdir(json_path):
        try:
            if os.path.isfile( os.path.join(json_path, file) ):
                # read from file
                with open( os.path.join(json_path, file), 'r' ) as fp:
                    # print "log: ", log
                    for _, line in enumerate(fp):
                        parsed_json = json.loads(line)
                        # print parsed_json[trajectories_source]
                        if parsed_json[trajectories_source]:
                            log_list = parsed_json[trajectories_source][trajectories_idx][trajectories_entry]

                            # for debugging
                            assert( not len(log_list) % 3)

                            log_list_grouped = grouped(log_list, grouping_factor)
                            keypoint_idx = 0
                            for x, y, prob in log_list_grouped:
                                x, y, prob = np.float64(x), np.float64(y), np.float64(prob)
                                # print body_25_body_parts_dict.get(keypoint_idx), x, y, prob
                                # first sanitize
                                if np.isnan(x): x = 0.0
                                if np.isnan(y): y = 0.0
                                if np.isnan(prob): prob = 0.0
                                report_matrix[keypoint_idx][log][ getKeysByValue(element_dict, "x")[0] ] = x
                                report_matrix[keypoint_idx][log][ getKeysByValue(element_dict, "y")[0] ] = y

                                keypoint_idx += 1

                log += 1

                if log == max_logs:
                    break

        except Exception as e:
            raise e

    # print report_matrix


    # write statistical analysis report
    for i in range(part):
        # write in the appropriate CSVs
        # x values
        with open(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_x" + ".csv", 'a') as fp:
            print >> fp , "t,x"
            for j in range(val):
                print >> fp , "t" + str(j) + "," + str(report_matrix[i][j][ getKeysByValue(element_dict, "x")[0] ])
        # y values
        with open(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_y" + ".csv", 'a') as fp:
            print >> fp , "t,y"
            for j in range(val):
                print >> fp , "t" + str(j) + "," + str(report_matrix[i][j][ getKeysByValue(element_dict, "y")[0] ])


    # all keypoints timeseries
    with open(trajectories_csvs_path + "all_keypoints_timeseries" + ".csv", 'a') as fp:
        for i in range(part):
            print >> fp, body_25_body_parts_dict.get(i)
            print >> fp, "x:" + ",".join( [ str(report_matrix[i][j][ getKeysByValue(element_dict, "x")[0] ]) for j in range(val) ] )
            print >> fp, "y:" + ",".join( [ str(report_matrix[i][j][ getKeysByValue(element_dict, "y")[0] ]) for j in range(val) ] )


    # set timeseries data specifications
    header_x, header_y = ['t', 'x'], ['t', 'y']
    dtypes_x, dtypes_y = {'t':'str', 'x':'float64'}, {'t':'str', 'y':'float64'}


    # create timeseries figures
    all_series_x, all_series_y = DataFrame(), DataFrame()
    for i in range(part):
        series_x = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_x" + ".csv", names=header_x, dtype=dtypes_x, header=0)
        series_y = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_y" + ".csv", names=header_y, dtype=dtypes_y, header=0)

        x_list, y_list = series_x['x'].values.tolist(), series_y['y'].values.tolist()
        # print x_list, y_list

        # timeseries plot trajectory
        plot(
            x_data=x_list,
            y_data=y_list,
            x_label="X Coord.",
            y_label="Y Coord.",
            title=body_25_body_parts_dict.get(i) + " trajectory plot",
            path=trajectories_plots_trajectories_path + body_25_body_parts_dict.get(i) + "_trajectory" + ".png"
        )

        # timeseries line plots
        lineplot(
            data=series_x,
            x_label="Instance",
            y_label="X coord.",
            title=body_25_body_parts_dict.get(i) + " x coordinate line plot",
            path=trajectories_plots_lines_path + body_25_body_parts_dict.get(i) + "_x" + "_line" + ".png"
        )
        lineplot(
            data=series_y,
            x_label="Instance",
            y_label="Y coord.",
            title=body_25_body_parts_dict.get(i) + " y coordinate line plot",
            path=trajectories_plots_lines_path + body_25_body_parts_dict.get(i) + "_y" + "_line" + ".png"
        )

        # timeseries histogram and density plots
        # histograms
        histogram(
            data=series_x,
            x_label="X coord.",
            y_label="Num. of occurences",
            title=body_25_body_parts_dict.get(i) + " x coordinate histogram",
            path=trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_x" + "_hist" + ".png"
        )
        histogram(
            data=series_y,
            x_label="Y coord.",
            y_label="Num. of occurences",
            title=body_25_body_parts_dict.get(i) + " y coordinate histogram",
            path=trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_y" + "_hist" + ".png"
        )
        # densities
        try:
            density_plot(
                data=series_x,
                x_label="X coord.",
                y_label="Density of values",
                title=body_25_body_parts_dict.get(i) + " x coordinate density plot",
                path=trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_x_dens" + ".png"
            )
            density_plot(
                data=series_y,
                x_label="Y coord.",
                y_label="Density of values",
                title=body_25_body_parts_dict.get(i) + " y coordinate density plot",
                path=trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_y_dens" + ".png"
            )
        except Exception as e:
            if e.__class__.__name__ == "LinAlgError":
                # source: https://stats.stackexchange.com/questions/89754/statsmodels-error-in-kde-on-a-list-of-repeated-values
                '''
                set_bandwidth() for kde is most probably not working, because of too many repeated instances of the same number.
                In such a case, we are feeding the code a distribution whose standard deviation is zero,
                and the code is attempting to use this value to calculate an initial guess for the KDE bandwidth parameter,
                and it's choking because zero isn't really a valid value.
                So, we do not plot and we just continue.
                '''
                continue
            raise e
        
        # for timeseries boxplots
        all_series_x[ i ] = [ e[1] for e in series_x.values.tolist() ]
        all_series_y[ i ] = [ e[1] for e in series_y.values.tolist() ]

        # # debugging
        # print(series_x.head())
        # print(series_y.head())


    # timeseries plot trajectories among pairs
    for i in range(part):
        series_x = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_x" + ".csv", names=header_x, dtype=dtypes_x, header=0)
        series_y = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_y" + ".csv", names=header_y, dtype=dtypes_y, header=0)

        x_list, y_list = series_x['x'].values.tolist(), series_y['y'].values.tolist()

        x_values, y_values = [], []
        x_values.append(x_list)
        y_values.append(y_list)

        # if keypoint with ID i has pairs
        if i in body_25_body_part_pairs_dict.keys():
            keypoint_names = []
            keypoint_names.append(body_25_body_parts_dict.get(i))

            paired_keypoints = body_25_body_part_pairs_dict.get(i)
            # print body_25_body_parts_dict.get(i) + " pairs with: " + ", ".join([ body_25_body_parts_dict.get(kp) for kp in paired_keypoints ])

            # for every paired keypoint with ID j
            for j in paired_keypoints:
                series_x = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(j) + "_x" + ".csv", names=header_x, dtype=dtypes_x, header=0)
                series_y = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(j) + "_y" + ".csv", names=header_y, dtype=dtypes_y, header=0)

                x_list, y_list = series_x['x'].values.tolist(), series_y['y'].values.tolist()
                x_values.append(x_list)
                y_values.append(y_list)
                
                keypoint_names.append(body_25_body_parts_dict.get(j))
            
                multiplot(
                    x_data=x_values,
                    y_data=y_values,
                    data_names=keypoint_names,
                    x_label="X Coord.",
                    y_label="Y Coord.",
                    title=body_25_body_parts_dict.get(j) + " paired with: " + ", ".join([ body_25_body_parts_dict.get(kp) for kp in paired_keypoints ]) + " trajectory plots",
                    path=trajectories_plots_trajectories_path + body_25_body_parts_dict.get(i) + "_&_" + "_".join([ body_25_body_parts_dict.get(kp) for kp in paired_keypoints ]) +"_trajectories" + ".png"
                )


    # timeseries boxplots
    # print all_series_x, all_series_y
    boxplot(
        data=all_series_x,
        x_label="Keypoint ID",
        y_label="X coord. value",
        title="All keypoints x coordinate boxplot",
        path=trajectories_plots_boxplots_path + "all_keypoints_x" + "_boxplot" + ".png"
    )
    boxplot(
        data=all_series_y,
        x_label="Keypoint ID",
        y_label="Y coord. value",
        title="All keypoints y coordinate boxplot",
        path=trajectories_plots_boxplots_path + "all_keypoints_y" + "_boxplot" + ".png"
    )


    print "SUCCESS!"