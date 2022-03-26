import os
import sys
import numpy as np
from PIL import Image


if __name__ == '__main__':
    rootdir = '../../maps/city/'

    for path, dirs, files in os.walk(rootdir):
        for file in files:
            if 'svg' == file.split('.')[1]:
                continue

            fullpath = os.path.join(path, file)
            with open(fullpath) as f:
                line = f.readline()
                line = f.readline()

                height = int(line.split(' ')[1])
                line = f.readline()

                width = int(line.split(' ')[1])
                line = f.readline()

                mapdata = np.array([list(line.rstrip()) for line in f])

            mapdata.reshape((width,height))
            mapdata[mapdata == '.'] = 0
            mapdata[mapdata == '@'] = 1
            mapdata = mapdata.astype(int)

            border = np.array([1 for _ in range(0, width)])
            mapdata = np.insert(mapdata, 0, border, axis=0)
            mapdata = np.insert(mapdata, height+1, border, axis=0)

            border = np.array([1 for _ in range(0, height+2)])
            mapdata = np.insert(mapdata, 0, border, axis=1)
            mapdata = np.insert(mapdata, width+1, border, axis=1)

            width += 2
            height += 2

            # print modified map file
            new_file = rootdir + ((fullpath.split('.')[2]).split('/')[3]) + '_modified.map'
            with open(new_file, 'w') as f:
                f.write("type octile\n")
                f.write("height " + str(height) + "\n")
                f.write("width " + str(width) + "\n")
                f.write("map\n")

                for r in range(height):
                    line = ""
                    for c in range(width):
                        # print('Color: {}'.format(img_matrix[x,y]))
                        char = 'O' if mapdata[r, c] else 'N'
                        line += char
                    line += "\n"
                    f.write(line)
