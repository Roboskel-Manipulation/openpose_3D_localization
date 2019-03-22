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

    # OpenPose specific variables
    element_dict = dict([ (0, "x"), (1, "y")])

    # File I/O specific variables
    json_dir = "/home/gkamaras/catkin_ws/src/openpose_ros/openpose_ros_receiver/output/SCENARIOS/json/samples/"
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


    # create timeseries figures
    header_x, header_y = ['t', 'x'], ['t', 'y']
    dtypes_x, dtypes_y = {'t':'str', 'x':'float64'}, {'t':'str', 'y':'float64'}
    all_series_x, all_series_y = DataFrame(), DataFrame()
    for i in range(part):
        series_x = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_x" + ".csv", names=header_x, dtype=dtypes_x, header=0)
        series_y = pd.read_csv(trajectories_csvs_path + body_25_body_parts_dict.get(i) + "_y" + ".csv", names=header_y, dtype=dtypes_y, header=0)
        # timeseries line plots
        series_x.plot()
        plt.savefig(trajectories_plots_lines_path + body_25_body_parts_dict.get(i) + "_x" + "_line" + ".png")
        plt.close()
        series_y.plot()
        plt.savefig(trajectories_plots_lines_path + body_25_body_parts_dict.get(i) + "_y" + "_line" + ".png")
        plt.close()
        # timeseries histogram and density plots
        # histograms
        series_x.hist()
        plt.savefig(trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_x" + "_hist" + ".png")
        plt.close()
        series_y.hist()
        plt.savefig(trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_y" + "_hist" + ".png")
        plt.close()
        # densities
        try:
            series_x.plot(kind='kde')
            plt.savefig(trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_x_dens" + ".png")
            plt.close()
            series_y.plot(kind='kde')
            plt.savefig(trajectories_plots_histograms_path + body_25_body_parts_dict.get(i) + "_y_dens" + ".png")
            plt.close()
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
        # debugging
        print(series_x.head())
        print(series_y.head())

    # timeseries boxplots
    print all_series_x, all_series_y
    all_series_x.boxplot()
    plt.savefig(trajectories_plots_boxplots_path + "all_keypoints_x" + "_boxplot" + ".png")
    plt.close()
    all_series_y.boxplot()
    plt.savefig(trajectories_plots_boxplots_path + "all_keypoints_y" + "_boxplot" + ".png")
    plt.close()

    print "SUCCESS!"