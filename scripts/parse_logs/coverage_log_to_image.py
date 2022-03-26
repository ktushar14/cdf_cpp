import sys
import pandas as pd
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


if __name__ == '__main__':
    logfile = sys.argv[1]
    mapfile = sys.argv[2]

    df = pd.read_csv(logfile)
    X = df['x']
    Y = df['y']

    ##############
    ## Read map ##
    ##############
    with open(mapfile) as f:
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

    unique, counts = np.unique(mapdata_ints, return_counts=True)
    print(unique)

    # assert(len(unique) == 3)
    # total_uncovered = counts[2]
    # print("total_uncovered:")
    # print(total_uncovered)

    # create discrete colormap
    map_cmap = mpl.colors.ListedColormap(['black','green','white'])
    # bounds = [0,10,20]
    # norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    # ax.imshow(data, cmap=cmap, norm=norm)
    ax.imshow(mapdata_ints, cmap=map_cmap)

    # draw gridlines
    # ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    # ax.set_xticks(np.arange(0, width, 2));
    # ax.set_yticks(np.arange(0, height, 2));

    colors = range(0, len(X))

    # ax.plot(X, Y, c='grey', linewidth=1, alpha=0.8, linestyle='-')
    # ax.plot(X, Y, c=colors, cmap='Spectral', linewidth=0.8, alpha=0.8, linestyle='-')
    # points_plot = ax.scatter(X, Y, c=colors, cmap='viridis', s=12)
    # points_plot = ax.scatter(X, Y, s=12)
    # plt.colorbar(points_plot)

    ## only coverage path single line
    ax.plot(X, Y, c='black', linewidth=0.5, alpha=1)
    # ax.grid()

    plt.gca().set_aspect("equal")
    plt.show()
    # imgname = mapfile + '_IMG.png'
    # plt.savefig(imgname, dpi=600)
