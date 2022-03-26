# file tools
import os
import sys

# plotting tools
import matplotlib as mpl
import matplotlib.pyplot as plt

# Core
import math
import numpy as np
import pandas as pd


kPatternAngularVel = 0.14
kTvMax = 8.0

def trans_vel_from_length(length):
    if ((length > 0) and (length <= 2.0)):
        return 1.0
    elif ((length > 2.0) and (length <= 4.0)):
        return 2.0
    elif ((length > 4.0) and (length <= 6.0)):
        return 4.0
    elif ((length > 6.0) and (length <= 8.0)):
        return 6.0
    elif length > 8.0:
        return 8.0

    # if length > kTvMax:
    #     return kTvMax
    # else:
    #     return float(np.floor(length))


def get_pattern_exec_times(n_straight_data, L, n_arc_data, DTheta):

    # print(len(n_straight_data))
    # print(len(L))
    # print(len(n_arc_data))
    # print(len(DTheta))

    assert(len(n_straight_data) == len(L))
    assert(len(L) == len(n_arc_data))
    assert(len(n_arc_data) == len(DTheta))

    times = []

    # calculate exec time for patterns in each iteration and store in times[]
    for i in range(len(L)):
        n_straight_segments = n_straight_data[i]
        n_arc_segments = n_arc_data[i]

        # zero execution time if single-cell sense
        if n_straight_segments == 0 and n_arc_segments == 0:
            times.append(0.0)
            continue

        # pattern with non-zero width or height
        T = 0.0

        # stright segments
        assert(n_straight_segments > 0)
        l = L[i]
        tv = trans_vel_from_length(l)
        assert(tv > 0 and tv <= kTvMax)
        T += n_straight_segments * (l / tv)

        # arc segments
        if not n_arc_segments == 0:
            dtheta = DTheta[i]
            av = kPatternAngularVel
            T += n_arc_segments * (dtheta / av)

        times.append(T)

    return np.array(times)


def main(rootdir):

    ########################
    # All experiments data #
    ########################
    ALLEXPS_FB_MEAN_planning_times_s = np.array([], dtype=float)
    ALLEXPS_FB_MEAN_exec_times_s = np.array([], dtype=float)
    ALLEXPS_FB_TOTAL_planning_times_s = np.array([], dtype=float)
    ALLEXPS_FB_TOTAL_exec_times_s = np.array([], dtype=float)

    ALLEXPS_OURS_MEAN_planning_times_s = np.array([], dtype=float)
    ALLEXPS_OURS_MEAN_exec_times_s = np.array([], dtype=float)
    ALLEXPS_OURS_TOTAL_planning_times_s = np.array([], dtype=float)
    ALLEXPS_OURS_TOTAL_exec_times_s = np.array([], dtype=float)


    for path, dirs, files in os.walk(rootdir):

        #########################
        # One single experiment #
        #########################
        if 'timing.txt' in files:

            # map and pattern info
            map_type = path.split('/')[2] # e.g.: walls_and_gaps
            map_name = path.split('/')[3] # e.g.: 10
            pattern_type = path.split('/')[4]

            # print('map_type + map_name: {}'.format(str(map_type + ' ' + map_name)))

            #####################
            # Read the log file #
            #####################
            # logfile_name = files[2] # files = {'coverage.txt', 'info.txt', 'timing.txt'}
            logfile_name = files[1] # files = {'coverage.txt', 'timing.txt'}
            # print(os.path.join(path, logfile_name))
            df = pd.read_csv(os.path.join(path, logfile_name))
            iterations = np.array(df['iteration'])
            planning_times_s = np.array(df['planning_time_ms'], dtype=float) / 1e3
            move_exec_times_s = np.array(df['move_exec_time_ms'], dtype=float) / 1e3

            pattern_exec_times_s = []
            if pattern_type == 'frontier_only':
                pattern_exec_times_s = np.array([0.0 for _ in range(0, len(iterations))])
            else:
                n_straight_data = np.array(df['n_straight'])
                n_arc_data = np.array(df['n_arc'])
                L = np.array(df['l'])
                DTheta = np.array(df['dtheta'])
                pattern_exec_times_s = get_pattern_exec_times(n_straight_data, L, n_arc_data, DTheta)

            assert(len(pattern_exec_times_s) == len(move_exec_times_s))
            exec_times_s = move_exec_times_s + pattern_exec_times_s

            ##########################################
            # Sum up individual times, get mean, std #
            ##########################################
            SUM_planning_time_s = np.sum(planning_times_s)
            SUM_exec_time_s = np.sum(exec_times_s)
            # SUM_planning_and_exec_time_s = SUM_planning_time_s + SUM_exec_time_s

            MEAN_planning_time_s = np.mean(planning_times_s)
            STD_planning_time_s = np.std(planning_times_s)

            MEAN_exec_time_s = np.mean(exec_times_s)
            STD_exec_time_s = np.std(exec_times_s)

            ###########################
            # Store in all exps array #
            ###########################

            # planning times
            if pattern_type == 'frontier_only':
                ALLEXPS_FB_MEAN_planning_times_s = np.append(ALLEXPS_FB_MEAN_planning_times_s, MEAN_planning_time_s)
                ALLEXPS_FB_TOTAL_planning_times_s = np.append(ALLEXPS_FB_TOTAL_planning_times_s, SUM_planning_time_s)
            else:
                print(MEAN_planning_time_s)
                ALLEXPS_OURS_MEAN_planning_times_s = np.append(ALLEXPS_OURS_MEAN_planning_times_s, MEAN_planning_time_s)
                ALLEXPS_OURS_TOTAL_planning_times_s = np.append(ALLEXPS_OURS_TOTAL_planning_times_s, SUM_planning_time_s)

            # exec times
            if pattern_type == 'frontier_only':
                ALLEXPS_FB_MEAN_exec_times_s = np.append(ALLEXPS_FB_MEAN_exec_times_s, MEAN_exec_time_s)
                ALLEXPS_FB_TOTAL_exec_times_s = np.append(ALLEXPS_FB_TOTAL_exec_times_s, SUM_exec_time_s)
            else:
                ALLEXPS_OURS_MEAN_exec_times_s = np.append(ALLEXPS_OURS_MEAN_exec_times_s, MEAN_exec_time_s)
                ALLEXPS_OURS_TOTAL_exec_times_s = np.append(ALLEXPS_OURS_TOTAL_exec_times_s, SUM_exec_time_s)

            total_iterations = iterations[-1]

            # print('map = {}'.format(str(map_type + ' ' + map_name)))
            # print('    pattern = {}'.format(pattern_type))
            # print('        avg plan time (s) = {:.2f} +- {:.2f}'.format(MEAN_planning_time_s, STD_planning_time_s))
            # print('        avg exec time (s) = {:.2f} +- {:.2f}'.format(MEAN_exec_time_s, STD_exec_time_s))
            # print('\n')
            # print('        total plan time (s) = {:.2f}'.format(SUM_planning_time_s))
            # print('        total exec time (s) = {:.2f}'.format(SUM_exec_time_s))
            # # print('        total plan+exec (s) = {:.2f}'.format(SUM_planning_and_exec_time_s))
            # print('        iterations    = {}'.format(total_iterations))
            # print('\n')

    #######################################################
    # Plot for this set of experiments (this type of map) #
    #######################################################

    # print('ALLEXPS_OURS_MEAN_exec_times_s: {}'.format(ALLEXPS_OURS_MEAN_exec_times_s))
    # print('ALLEXPS_FB_MEAN_exec_times_s: {}'.format(ALLEXPS_FB_MEAN_exec_times_s))
    # print('ALLEXPS_OURS_TOTAL_exec_times_s: {}'.format(ALLEXPS_OURS_TOTAL_exec_times_s))
    # print('ALLEXPS_OURS_TOTAL_planning_times_s: {}'.format(ALLEXPS_OURS_TOTAL_planning_times_s))
    # print('\n')
    # print('ALLEXPS_FB_TOTAL_exec_times_s: {}'.format(ALLEXPS_FB_TOTAL_exec_times_s))
    # print('ALLEXPS_FB_TOTAL_planning_times_s: {}'.format(ALLEXPS_FB_TOTAL_planning_times_s))

    # plt.scatter(np.arange(1, len(ALLEXPS_OURS_MEAN_exec_times_s)+1, 1), ALLEXPS_OURS_MEAN_exec_times_s, c='r')
    # plt.scatter(np.arange(1, len(ALLEXPS_FB_MEAN_exec_times_s)+1, 1), ALLEXPS_FB_MEAN_exec_times_s, c='k')

    # plt.scatter(np.arange(1, len(ALLEXPS_OURS_TOTAL_exec_times_s)+1, 1), ALLEXPS_OURS_TOTAL_exec_times_s, c='r')
    # plt.scatter(np.arange(1, len(ALLEXPS_FB_TOTAL_exec_times_s)+1, 1), ALLEXPS_FB_TOTAL_exec_times_s, c='k')
    # plt.show()

    ###################
    ## MEAN OF MEANS ##
    ###################

    MEAN_OURS_MEAN_planning_times = np.mean(ALLEXPS_OURS_MEAN_planning_times_s, axis=0)
    STD_OURS_MEAN_planning_times = np.std(ALLEXPS_OURS_MEAN_planning_times_s, axis=0)

    MEAN_FB_MEAN_planning_times = np.mean(ALLEXPS_FB_MEAN_planning_times_s, axis=0)
    STD_FB_MEAN_planning_times = np.std(ALLEXPS_FB_MEAN_planning_times_s, axis=0)

    MEAN_OURS_MEAN_exec_times = np.mean(ALLEXPS_OURS_MEAN_exec_times_s)
    MEAN_FB_MEAN_exec_times = np.mean(ALLEXPS_FB_MEAN_exec_times_s)

    MEAN_OURS_TOTAL_planning_times = np.mean(ALLEXPS_OURS_TOTAL_planning_times_s)
    STD_OURS_TOTAL_planning_times = np.std(ALLEXPS_OURS_TOTAL_planning_times_s)
    MEAN_FB_TOTAL_planning_times = np.mean(ALLEXPS_FB_TOTAL_planning_times_s)
    STD_FB_TOTAL_planning_times = np.std(ALLEXPS_FB_TOTAL_planning_times_s)

    MEAN_OURS_TOTAL_exec_times = np.mean(ALLEXPS_OURS_TOTAL_exec_times_s)
    STD_OURS_TOTAL_exec_times = np.std(ALLEXPS_OURS_TOTAL_exec_times_s)
    MEAN_FB_TOTAL_exec_times = np.mean(ALLEXPS_FB_TOTAL_exec_times_s)
    STD_FB_TOTAL_exec_times = np.std(ALLEXPS_FB_TOTAL_exec_times_s)

    print('\n')
    print('MEAN_OURS_MEAN_planning_times: {}'.format(MEAN_OURS_MEAN_planning_times))
    print('STD_OURS_MEAN_planning_times: {}'.format(STD_OURS_MEAN_planning_times))
    print('MEAN_FB_MEAN_planning_times: {}'.format(MEAN_FB_MEAN_planning_times))
    print('STD_FB_MEAN_planning_times: {}'.format(STD_FB_MEAN_planning_times))
    # print('\n')
    # print('MEAN_OURS_MEAN_exec_times: {}'.format(MEAN_OURS_MEAN_exec_times))
    # print('MEAN_FB_MEAN_exec_times: {}'.format(MEAN_FB_MEAN_exec_times))
    print('\n')
    print('MEAN_OURS_TOTAL_planning_times: {}'.format(int(MEAN_OURS_TOTAL_planning_times)))
    print('STD_OURS_TOTAL_planning_times: {}'.format(int(STD_OURS_TOTAL_planning_times)))
    print('MEAN_FB_TOTAL_planning_times: {}'.format(int(MEAN_FB_TOTAL_planning_times)))
    print('STD_FB_TOTAL_planning_times: {}'.format(int(STD_FB_TOTAL_planning_times)))
    print('\n')
    print('MEAN_OURS_TOTAL_exec_times: {}'.format(int(MEAN_OURS_TOTAL_exec_times)))
    print('STD_OURS_TOTAL_exec_times: {}'.format(int(STD_OURS_TOTAL_exec_times)))
    print('MEAN_FB_TOTAL_exec_times: {}'.format(int(MEAN_FB_TOTAL_exec_times)))
    print('STD_FB_TOTAL_exec_times: {}'.format(int(STD_FB_TOTAL_exec_times)))

if __name__=='__main__':
    assert(len(sys.argv) == 2)
    rootdir = sys.argv[1]
    main(rootdir)
