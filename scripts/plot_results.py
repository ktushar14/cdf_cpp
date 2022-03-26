##
## Execute as py plot_results ../../results/maps/ patterns_30_30 random
##

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

    ## PERCENTAGE
    percent_coverage = 100 * cum_cells_covered * (1/total_coverage_cells)
    # return cum_length_traveled, percent_coverage

    ## NUMBER OF CELLS
    return cum_length_traveled, cum_cells_covered


@dataclass
class ApproachData:
    clt: np.ndarray = np.array([], dtype=float)
    pc:  np.ndarray = np.array([], dtype=float)
    approach: str = ""
    map_type: str = ""
    # map_name: str = ""


if __name__=='__main__':

    rootdir = sys.argv[1]
    approach1 = sys.argv[2]
    approach2 = sys.argv[3]
    approach3 = sys.argv[4]
    print("approach1: {}".format(approach1))
    print("approach2: {}".format(approach2))
    print("approach3: {}".format(approach3))

    all_data = []
    all_map_types = set()
    all_pattern_types = set()

    # app2clt = {} # dict: 'approach' -> cum_length_traveled
    # app2pc  = {} # dict: 'approach' -> percent coverage

    for path, dirs, files in os.walk(rootdir):
        if 'coverage.txt' in files:
            print("dir: {}".format(path))
            data = ApproachData()

            map_type = path.split('/')[1]
            # map_name = path.split('/')[2]
            data.approach = path.split('/')[4]

            # print(path)
            print(data.map_type)
            print(data.approach)

            log_file_name = files[0]
            log_file_path = os.path.join(path, log_file_name)
            data.clt, data.pc = parse_log_file(log_file_path)

            # app2clt[approach] = clt
            # app2pc[approach]  = pc

            all_data.append(data)

    # Plot
    fig, ax = plt.subplots()
    for y in np.arange(0, 100+20, 20):
        plt.axhline(y=y, color='k', linestyle='-', linewidth=1, alpha=0.5)

    plt.title(r'\emph{percent coverage v/s distance traveled}')
    plt.ylabel(r'\emph{coverage (\%)}')
    plt.xlabel(r'\emph{distance traveled}')

    for data in all_data:
        X = np.array(data.clt)
        Y = np.array(data.pc)

        X_subsample = X[::100]
        Y_subsample = Y[::100]
        # X_subsample = X
        # Y_subsample = Y

        # OURS
        if data.approach == approach1:
            ax.errorbar(
                X_subsample, Y_subsample,
                xerr=100,
                # '-o',
                # X, Y,
                # linewidth=1,
                c='red',
                alpha=1)
        # RANDOM
        elif data.approach == approach2:
            ax.errorbar(
                X_subsample, Y_subsample,
                xerr=100,
                # '-o',
                # X, Y,
                # linewidth=1,
                c='blue',
                alpha=1)
        # FRONTIER
        elif data.approach == approach3:
            ax.errorbar(
                X_subsample, Y_subsample,
                xerr=100,
                # '-o',
                # X, Y,
                # linewidth=1,
                c='green',
                alpha=1)

    plt.show()