import sys
import pandas as pd
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif":  ["Palatino"],
    "font.size":   14
})

if __name__ == '__main__':

    ##############
    # FILE PATHS #
    ##############

    log_folder = sys.argv[1]  # ../results/walls_and_gaps/1/
    pattern_type = sys.argv[2]  # patterns_30_30

    # ../results/walls_and_gaps/1/frontier_only/coverage.txt
    coverage_logfile_ours = log_folder + pattern_type + '/coverage.txt'
    print('coverage_logfile_ours = {}'.format(coverage_logfile_ours))

    # ../results/walls_and_gaps/1/patterns_30_30/coverage.txt
    coverage_logfile_base = log_folder + 'frontier_only/coverage.txt'
    print('coverage_logfile_base = {}'.format(coverage_logfile_base))

    # ../maps/walls_and_gaps/1.map
    map_file = '../maps/' + log_folder.split('/')[2] + '/' + log_folder.split('/')[3] + '.map'

    #########
    # PARSE #
    #########

    # Read map
    with open(map_file) as f:
        line = f.readline()
        line = f.readline()
        height = int(line.split(' ')[1])
        line = f.readline()
        width = int(line.split(' ')[1])
        line = f.readline()
        mapdata = np.array([list(line.rstrip()) for line in f])

    # print(mapdata)
    mapdata_ints = mapdata

    mapdata_ints.reshape((height, width))
    mapdata_ints[mapdata_ints == 'O'] = 0
    mapdata_ints[mapdata_ints == 'C'] = 1
    mapdata_ints[mapdata_ints == 'N'] = 2
    mapdata_ints = mapdata_ints.astype(int)

    # Read coverage logs
    df_ours = pd.read_csv(coverage_logfile_ours)
    X_ours = df_ours['x']
    Y_ours = df_ours['y']
    Covered_ours = df_ours['covered']
    Length_ours = df_ours['length']

    df_base = pd.read_csv(coverage_logfile_base)
    X_base = df_base['x']
    Y_base = df_base['y']
    Covered_base = df_base['covered']
    Length_base = df_base['length']

    n_points_ours = len(X_ours)
    n_points_base = len(X_base)

    n_total = np.max([n_points_base, n_points_ours])

    fig, axs = plt.subplots(nrows=2, ncols=2)
    map_cmap = mpl.colors.ListedColormap(['black','green','white'])

    ax_ours = plt.subplot(2,2,1)
    ax_ours.set_title(r'\emph{Ours}')
    ax_ours.imshow(mapdata_ints, cmap=map_cmap)

    ax_base = plt.subplot(2,2,2)
    ax_base.set_title(r'\emph{Frontier-based coverage}')
    ax_base.imshow(mapdata_ints, cmap=map_cmap)

    ax_plot = plt.subplot(2,1,2)
    ax_plot.set_title(r'\emph{Coverage versus distance traveled}')
    ax_plot.set_xlabel(r'Distance')
    ax_plot.set_ylabel(r'\# cells covered')

    print('n_total = {}'.format(n_total))

    length_ours_arr = []
    covered_ours_arr = []
    length_base_arr = []
    covered_base_arr = []

    covered_X_ours = []
    covered_Y_ours = []
    covered_X_base = []
    covered_Y_base = []

    for i in np.arange(0, n_total, 1):
        # print('i = {}'.format(i))
        # ours
        if i < n_points_ours:
            x_ours = X_ours[i]
            y_ours = Y_ours[i]
            covered_ours = Covered_ours[i]
            length_ours = Length_ours[i]
        else:
            x_ours = X_ours[-1]
            y_ours = Y_ours[-1]
            covered_ours = Covered_ours[-1]
            length_ours = Length_ours[-1]
        length_ours_arr.append(length_ours)
        covered_ours_arr.append(covered_ours)
        covered_X_ours.append(x_ours)
        covered_Y_ours.append(y_ours)

        # base
        if i < n_points_base:
            x_base = X_base[i]
            y_base = Y_base[i]
            covered_base = Covered_base[i]
            length_base = Length_base[i]
        else:
            x_base = X_base[-1]
            y_base = Y_base[-1]
            covered_base = Covered_base[-1]
            length_base = Length_base[-1]
        length_base_arr.append(length_base)
        covered_base_arr.append(covered_base)
        covered_X_base.append(x_base)
        covered_Y_base.append(y_base)

        if (i < 8000 and i % 200 == 0) or (i > 8000 and i % 50 == 0):

            # plot everything
            ax_ours.scatter(covered_X_ours, covered_Y_ours, s=1, c='r')
            ax_plot.plot(length_ours_arr, covered_ours_arr, c='r')

            ax_base.scatter(covered_X_base, covered_Y_base, s=1, c='b')
            ax_plot.plot(length_base_arr, covered_base_arr, c='b')

            filename = 'video_imgs/img' + str(i).zfill(5) + '.png'
            plt.tight_layout()
            plt.savefig(filename, dpi=600)
            print("saved fig {}".format(i))

    i = n_total-1
    if i < n_points_ours:
        x_ours = X_ours[i]
        y_ours = Y_ours[i]
        covered_ours = Covered_ours[i]
        length_ours = Length_ours[i]
    else:
        x_ours = X_ours[-1]
        y_ours = Y_ours[-1]
        covered_ours = Covered_ours[-1]
        length_ours = Length_ours[-1]
    length_ours_arr.append(length_ours)
    covered_ours_arr.append(covered_ours)
    covered_X_ours.append(x_ours)
    covered_Y_ours.append(y_ours)

    # base
    if i < n_points_base:
        x_base = X_base[i]
        y_base = Y_base[i]
        covered_base = Covered_base[i]
        length_base = Length_base[i]
    else:
        x_base = X_base[-1]
        y_base = Y_base[-1]
        covered_base = Covered_base[-1]
        length_base = Length_base[-1]
    length_base_arr.append(length_base)
    covered_base_arr.append(covered_base)
    covered_X_base.append(x_base)
    covered_Y_base.append(y_base)

    # plot everything
    ax_ours.scatter(covered_X_ours, covered_Y_ours, s=1, c='r')
    ax_plot.plot(length_ours_arr, covered_ours_arr, c='r')

    ax_base.scatter(covered_X_base, covered_Y_base, s=1, c='b')
    ax_plot.plot(length_base_arr, covered_base_arr, c='b')

    filename = 'video_imgs/img' + str(i).zfill(5) + '.png'
    plt.tight_layout()
    plt.savefig(filename, dpi=600)
    print("saved fig {}".format(i))
