# file tools
import os
import sys

# plotting tools
import matplotlib as mpl
import matplotlib.pyplot as plt
# mpl.rcParams['ps.useafm'] = True
# mpl.rcParams['pdf.use14corefonts'] = True
# mpl.rcParams['text.usetex'] = True
# # plt.rcParams.update({'font.size': 45})

# mpl.rcParams['text.usetex'] = True
# mpl.rc('font',family='serif', serif=['Palatino'])
# plt.rcParams.update({'font.size': 12})

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif":  ["Palatino"],
    "font.size":   14
})

# Core
import math
import numpy as np
import pandas as pd
from dataclasses import dataclass
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import interp1d


@dataclass
class Data:
    cum_length_traveled: np.ndarray = np.array([], dtype=float)
    percent_coverage: np.ndarray = np.array([], dtype=float)
    pattern_type: str = ""
    map_type: str = ""
    map_name: str = ""


def enlarge_arr_if_needed(old_array, new_length):
    assert(~(len(old_array) > new_length))
    if len(old_array) == new_length:
        return old_array
    old_indices = np.arange(0, len(old_array))
    new_indices = np.linspace(0, len(old_array) - 1, new_length)
    spl = UnivariateSpline(old_indices, old_array, k = 3, s = 0)
    # spl = interp1d(old_indices, old_array)
    new_array = spl(new_indices)
    return new_array


def errorfill(x, y, yerr, color='blue', alpha_fill=0.3, ax=None):
    ax = ax if ax is not None else plt.gca()
    # if color is None:
    #     color = ax._get_lines.color_cycle.next()
    if np.isscalar(yerr) or len(yerr) == len(y):
        ymin = y - yerr
        ymax = y + yerr
    elif len(yerr) == 2:
        ymin, ymax = yerr
    ax.plot(x, y, color=color)
    ax.fill_between(x, ymax, ymin, color=color, alpha=alpha_fill)


def parse_log_file(file_path):
    df = pd.read_csv(file_path)
    cum_length_traveled = np.array(df['length'], dtype=float)
    cum_cells_covered = np.array(df['covered'], dtype=float)
    total_coverage_cells = cum_cells_covered[-1]

    ##############
    # PERCENTAGE #
    ##############
    # percent_coverage = 100 * cum_cells_covered * (1/total_coverage_cells)

    #####################
    ## NUMBER OF CELLS ##
    #####################
    percent_coverage = cum_cells_covered
    return cum_length_traveled, percent_coverage


if __name__=='__main__':

    assert(len(sys.argv) == 3)
    rootdir = sys.argv[1]
    pattern_used = sys.argv[2] # ONLY NAME, not path (ex.: patterns_30_30)
    print("pattern_used: {}".format(pattern_used))

    all_data = []
    all_map_types = set()
    all_pattern_types = set()

    ############################################################################
    # PARSE FILES #
    ############################################################################

    max_clt_length_FONLY = 0
    max_clt_val_FONLY = 0
    all_pc_FONLY = []
    all_clt_FONLY = []

    max_clt_length_OURS = 0
    max_clt_val_OURS = 0
    all_pc_OURS = []
    all_clt_OURS = []

    for path, dirs, files in os.walk(rootdir):
        if 'coverage.txt' in files:
            print("dir: {}".format(path))
            data = Data()

            data.map_type = path.split('/')[2]
            data.map_name = path.split('/')[3]
            # data.pattern_type = path.split('/')[4]
            data.pattern_type = path.split('/')[5]

            all_map_types.add(data.map_type)
            all_pattern_types.add(data.pattern_type)

            log_file_name = files[0]
            log_file_path = os.path.join(path, log_file_name)

            clt, pc = parse_log_file(log_file_path)
            data.percent_coverage = pc
            data.cum_length_traveled = clt

            all_data.append(data)

            if data.pattern_type=='frontier_only':
                if len(clt) > max_clt_length_FONLY:
                    max_clt_length_FONLY = len(clt)
                if np.max(clt) > max_clt_val_FONLY:
                    max_clt_val_FONLY = np.max(clt)

            elif data.pattern_type==pattern_used:
                if len(clt) > max_clt_length_OURS:
                    max_clt_length_OURS = len(clt)
                if np.max(clt) > max_clt_val_OURS:
                    max_clt_val_OURS = np.max(clt)

            else:
                print('data.pattern_type: {}'.format(data.pattern_type))
                print('INVALID PATTERN TYPE !?')

    all_pattern_types = sorted(all_pattern_types)
    all_map_types = sorted(all_map_types)

    # Plot
    fig, ax = plt.subplots()
    for y in np.arange(0, 100+20, 20):
        plt.axhline(y=y, color='k', linestyle='-', linewidth=1, alpha=0.5)

    plt.title(r'\emph{percent coverage v/s distance traveled}')
    plt.ylabel(r'\emph{coverage (\%)}')
    plt.xlabel(r'\emph{distance traveled}')

    ############################################################################
    # PLOT INDIVIDUAL TRAJECTORIES #
    ############################################################################
    for data in all_data:
        X = data.cum_length_traveled
        Y = data.percent_coverage

        if data.pattern_type == pattern_used:
            all_clt_OURS.append(X)
            all_pc_OURS.append(Y)
            # ax.plot(
            #     X, Y,
            #     linewidth=1,
            #     c='red',
            #     alpha=0.3)
        elif data.pattern_type == 'frontier_only':
            all_clt_FONLY.append(X)
            all_pc_FONLY.append(Y)
            # ax.plot(
            #     X, Y,
            #     linewidth=1,
            #     c='blue',
            #     alpha=0.3)
        else:
            print('data.pattern_type: {}'.format(data.pattern_type))
            print('INVALID PATTERN TYPE !?')

        # print('    {}'.format(data.map_type))
        # print('    {}'.format(data.map_name))
        # print('    {}'.format(data.pattern_type))
        # print('    {}'.format(data.cum_length_traveled[-1]))
        # print('    {}'.format(data.percent_coverage[-1]))

    ############################################################################
    # PLOT MEAN + STD
    ############################################################################

    ########
    # OURS #
    ########

    count = 0
    for i, data in enumerate(all_pc_OURS):
        count += 1
        clt = enlarge_arr_if_needed(all_clt_OURS[i], max_clt_length_OURS)
        all_clt_OURS[i] = clt
        pc = enlarge_arr_if_needed(all_pc_OURS[i], max_clt_length_OURS)
        all_pc_OURS[i] = pc

        assert(len(all_pc_OURS[i]) == max_clt_length_OURS)

        # ax.plot(
        #     all_clt_OURS[i], all_pc_OURS[i],
        #     linewidth=1,
        #     c='red',
        #     alpha=0.3)

    all_pc_OURS = np.resize(all_pc_OURS, (count, max_clt_length_OURS))
    mean_pc_arr = np.mean(all_pc_OURS, axis=0)
    std_pc_arr = np.std(all_pc_OURS, axis=0)

    all_clt_OURS = np.resize(all_clt_OURS, (count, max_clt_length_OURS))
    mean_clt_arr = np.mean(all_clt_OURS, axis=0)
    std_clt_arr = np.std(all_clt_OURS, axis=0)
    std_clt_arr = std_clt_arr

    ax.errorbar(mean_clt_arr,
                mean_pc_arr,
                xerr=std_clt_arr,
                capsize=2,
                errorevery=5000,
                color='red',
                ecolor='red')
    # ax.plot(mean_clt_arr, mean_pc_arr, color='red', linewidth=2)

    count = 0
    for i, data in enumerate(all_pc_FONLY):
        count += 1
        clt = enlarge_arr_if_needed(all_clt_FONLY[i], max_clt_length_FONLY)
        all_clt_FONLY[i] = clt
        pc = enlarge_arr_if_needed(all_pc_FONLY[i], max_clt_length_FONLY)
        all_pc_FONLY[i] = pc

        assert(len(all_pc_FONLY[i]) == max_clt_length_FONLY)

        # ax.plot(
        #     all_clt_FONLY[i], all_pc_FONLY[i],
        #     linewidth=1,
        #     c='blue',
        #     alpha=0.3)

    all_pc_FONLY = np.resize(all_pc_FONLY, (count, max_clt_length_FONLY))
    mean_pc_arr = np.mean(all_pc_FONLY, axis=0)
    std_pc_arr = np.std(all_pc_FONLY, axis=0)

    all_clt_FONLY = np.resize(all_clt_FONLY, (count, max_clt_length_FONLY))
    mean_clt_arr = np.mean(all_clt_FONLY, axis=0)
    std_clt_arr = np.std(all_clt_FONLY, axis=0)
    std_clt_arr = std_clt_arr

    ax.errorbar(mean_clt_arr,
                mean_pc_arr,
                xerr=std_clt_arr,
                capsize=2,
                errorevery=5000,
                color='blue',
                ecolor='blue')
    # ax.plot(mean_clt_arr, mean_pc_arr, color='blue', linewidth=2)

    # plt.show()
    filename = 'plot_' + rootdir.split('/')[2] + '_coverage.png'
    print(filename)
    # plt.axis('equal')
    plt.savefig(filename, dpi=600)
