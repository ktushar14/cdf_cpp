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


def get_random_int_neq_set(a, b, input_set):
    result = random.randint(a, b)
    while result in input_set:
        result = random.randint(a, b)
    return result


def constrained_sum_sample_pos_dividers(n, total):
    """Return dividers between a randomly chosen list of n positive integers
    summing to total. Each such list is equally likely to occur."""

    dividers = sorted(random.sample(range(1, total), n - 1))
    return dividers


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


def get_wall_rows(num_widths, total_width, min_dist_bw_walls, grid_rows):
    while True:
        rows = np.array(constrained_sum_sample_pos_dividers(num_widths, total_width))
        rows_test = np.concatenate((np.array([0]), rows))
        rows_test = np.concatenate((rows_test, np.array([grid_rows-2])))
        diffs = np.diff(rows_test)
        if (diffs > min_dist_bw_walls).all():
            break
    return rows


def get_passage_rows(wall_rows, grid_rows):
    passage_rows = []
    for i in range(len(wall_rows) - 1):
        rows_before = wall_rows[i]
        rows_after = wall_rows[i+1]
        rows_to_avoid = np.concatenate((rows_before, rows_after))
        pr = get_random_int_neq_set(2, grid_rows-2, rows_to_avoid)
        passage_rows.append(pr)
    return passage_rows


def main(dim):

    # get parameters based on map size
    grid_cols = dim
    grid_rows = dim

    if dim == 100:
        # all room widths + all passage sizes = dim - 2
        N = 6 # must be even (just cause I'm lazy)

        min_width = 10
        max_width = 30

        min_room_height = 10
        num_vertical_rooms = 4

        min_gap_size = 2
        max_gap_size = 8

    elif dim == 250:
        # all room widths + all passage sizes = dim - 2
        N = 6 # must be even (just cause I'm lazy)

        min_width = 30
        max_width = 50

        min_room_height = 40
        num_vertical_rooms = 4

        min_gap_size = 2
        max_gap_size = 20

    else:
        print('Currently only accounts for dim = 100, 250')
        sys.exit()

    # create directories
    map_dir = '../../maps/corridors_and_rooms_' + str(dim) + '_' + str(dim) + '/'
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

        assert(N % 2 == 0)  # just cause I'm lazy
        num_room_sets = int(N/2)

        all_widths = get_sizes(N, grid_cols-2, min_width, max_width)
        passage_sizes = all_widths[0::2]
        room_widths = all_widths[1::2]

        wall_rows = []

        for i in range(num_room_sets):
            rows = get_wall_rows(num_vertical_rooms, grid_rows-2, min_room_height, grid_rows)  # num rooms (vertically) per set
            wall_rows.append(rows)
        wall_rows = np.array(wall_rows)

        # passage_rows = [1, 10]
        passage_rows = get_passage_rows(wall_rows, grid_rows)
        passage_rows = np.concatenate((np.array([1]), passage_rows))

        # passage_sizes = [5, 10]
        # room_widths = [10, 13]

        p_ci = 1
        for i in range(len(passage_rows)):

            # Add passage
            pr = passage_rows[i]
            sz = passage_sizes[i]

            p_cf = p_ci + sz

            grid[:, p_ci : p_cf] = 1
            grid[pr, p_ci : p_cf] = 0

            # Add rooms + walls
            r_width = room_widths[i]
            for wr in wall_rows[i]:
                grid[wr, p_cf : p_cf + r_width] = 1  # add wall
                # min_gap_size = 2
                # max_gap_size = int(r_width/2)
                if (min_gap_size == max_gap_size):
                    max_gap_size += 1
                gap_size = get_random_int(min_gap_size, max_gap_size)
                gap_ci_low = p_cf + 1
                gap_ci_high = p_cf + 1 + r_width - max_gap_size
                gap_ci = get_random_int(gap_ci_low, gap_ci_high)
                gap_cf = gap_ci + gap_size
                grid[wr, gap_ci : gap_cf] = 0  # add gap in wall

            p_ci = p_cf + r_width

        fig = plt.figure()
        ax = plt.gca()

        # ax.set_xticks(np.arange(-0.5, grid_cols, 1));
        # ax.set_yticks(np.arange(-0.5, grid_rows, 1));
        # ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
        ax.set_aspect("equal")

        cmap = mpl.colors.ListedColormap(['white','black'])
        img = plt.imshow(grid, cmap=cmap)

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
        # plt.show()


if __name__ == '__main__':

    # Only accounting for square maps (rows = cols)
    assert(len(sys.argv) == 2)
    mapdim = int(sys.argv[1])
    main(mapdim)
