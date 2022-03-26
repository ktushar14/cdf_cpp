import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from enum import Enum # pip install enum34
import random


# Get random integer in range [a, b]
def get_random_int(a, b):
    return random.randint(a, b)


def constrained_sum_sample_pos(n, total):
    """Return a randomly chosen list of n positive integers summing to total.
    Each such list is equally likely to occur."""

    dividers = sorted(random.sample(range(1, total), n - 1))
    return [a - b for a, b in zip(dividers + [total], [0] + dividers)]


def get_sizes(num_sizes, total_size, min_size, max_size):
    while True:
        sizes = np.array(constrained_sum_sample_pos(num_sizes, total_size))
        # if (sizes > min_size).all():
        #     break
        if (sizes > min_size).all() and (sizes < max_size).all():
            break
    return sizes


def main(dim):

    # get parameters based on map size
    grid_cols = dim
    grid_rows = dim

    num_rooms_hor = 5
    num_rooms_ver = 5

    if dim == 100:
        min_room_width = 10
        max_room_width = 30

        min_room_height = 10
        max_room_height = 30

        min_gap_size = 1
        max_gap_size = 2

    elif dim == 250:
        min_room_width = 40
        max_room_width = 60

        min_room_height = 40
        max_room_height = 60

        min_gap_size = 2
        max_gap_size = 30

    else:
        print('Currently only accounts for dim = 100, 250')
        sys.exit()

    # create directories
    map_dir = '../../maps/walls_and_gaps_new_' + str(dim) + '_' + str(dim) + '/'
    map_imgs_dir = map_dir + 'img/'
    if not os.path.exists(map_dir):
        os.makedirs(map_dir)
    if not os.path.exists(map_imgs_dir):
        os.makedirs(map_imgs_dir)

    # generate and save maps
    num_maps = 5

    for mapcount in np.arange(1, num_maps+1, 1):
        grid = np.zeros((grid_rows, grid_cols))
        grid[0, :] = 1
        grid[:, 0] = 1
        grid[grid_rows - 1, :] = 1
        grid[:, grid_cols - 1] = 1

        # get random room widths that fit in the grid
        room_widths = get_sizes(num_rooms_hor, grid_cols-2, min_room_width, max_room_width)
        # print(room_widths)

        # get random room heights that fit in the grid
        room_heights = get_sizes(num_rooms_ver, grid_rows-2, min_room_height, max_room_height)
        # print(room_heights)

        num_walls_hor = num_rooms_hor - 1
        num_walls_ver = num_rooms_ver - 1

        # add all vertical walls
        col_idx = 0
        wall_col_indices = []
        for s in room_widths[:-1]:
            col_idx += s
            wall_col_indices.append(col_idx)
            grid[:, col_idx] = 1

        # add all horizontal walls
        row_idx = 0
        wall_row_indices = []
        for s in room_heights[:-1]:
            row_idx += s
            wall_row_indices.append(row_idx)
            grid[row_idx, :] = 1

        # add gaps in vertical walls
        wall_row_indices.insert(0, 0)
        wall_row_indices.append(grid_rows-2)
        for wc in wall_col_indices:
            for i in range(len(wall_row_indices[:-1])):
                gap_size = get_random_int(min_gap_size, max_gap_size)
                wr = wall_row_indices[i]
                wr_next = wall_row_indices[i+1]
                gap_ri_low = wr + 2
                gap_ri_high = wr_next - gap_size
                gap_ri = get_random_int(gap_ri_low, gap_ri_high)
                gap_rf = gap_ri + gap_size
                # print("gap_ri: {}".format(gap_ri))
                # print("gap_rf: {}".format(gap_rf))
                # print("max_gap_size: {}".format(max_gap_size))
                grid[gap_ri : gap_rf, wc] = 0

        # add gaps in horizontal walls
        wall_col_indices.insert(0, 0)
        wall_col_indices.append(grid_cols-2)
        for wr in wall_row_indices[1:]:
            for i in range(len(wall_col_indices[:-1])):
                gap_size = get_random_int(min_gap_size, max_gap_size)
                wc = wall_col_indices[i]
                wc_next = wall_col_indices[i+1]
                gap_ci_low = wc + 2
                gap_ci_high = wc_next - gap_size
                gap_ci = get_random_int(gap_ci_low, gap_ci_high)
                gap_cf = gap_ci + gap_size
                # print("wr: {}".format(wr))
                # print("gap_ci: {}".format(gap_ci))
                # print("gap_cf: {}".format(gap_cf))
                # print("max_gap_size: {}".format(max_gap_size))
                grid[wr, gap_ci : gap_cf] = 0

        cmap = mpl.colors.ListedColormap(['white','black'])
        img = plt.imshow(grid, cmap=cmap)
        # plt.show()

        mapname = '' + str(mapcount)
        # plt.savefig(map_dir + mapname + '.png', dpi=300)
        plt.axis('off')
        plt.savefig(map_imgs_dir + mapname + '.svg', format="svg", dpi=300)
        plt.savefig(map_imgs_dir + mapname + '.pdf', dpi=300)

        movingai_mapfile = map_dir + mapname + '.map'
        with open(movingai_mapfile, 'w') as f:
            f.write("type octile\n")
            f.write("height " + str(grid_rows) + "\n")
            f.write("width " + str(grid_cols) + "\n")
            f.write("map\n")

            for r in range(grid_rows):
                line = ""
                for c in range(grid_cols):
                    # print('Color: {}'.format(img_matrix[x,y]))
                    char = 'O' if grid[r, c] else 'N'
                    line += char
                line += "\n"
                f.write(line)


if __name__ == '__main__':

    # Only accounting for square maps (rows = cols)
    assert(len(sys.argv) == 2)
    mapdim = int(sys.argv[1])
    main(mapdim)
