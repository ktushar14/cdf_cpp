import sys
import numpy as np

if __name__ == '__main__':

    xmax = int(sys.argv[1])
    ymax = int(sys.argv[2])
    delta = int(sys.argv[3])

    xsizes = [x for x in np.arange(10, xmax, delta)]
    ysizes = [y for y in np.arange(10, ymax, delta)]

    xsizes.append(xmax)
    ysizes.append(ymax)

    # ALL DIM COMBINATIONS #####################################################
    print('ALL DIM COMBINATIONS:')
    count = 0
    for xsize in xsizes:
        for ysize in ysizes:
            if xsize == 1 or ysize == 1:
                continue
            count += 1
            # print('# {} : [{}] x {}]'.format(count, xsize, ysize))

    print(repr(np.array(xsizes)))
    print(len(xsizes))
    print(4*count + 1)

    # ONLY SQUARE PATTERNS #####################################################
    print('ONLY SQUARE PATTERNS:')
    count = 0
    for xsize in xsizes:
        for ysize in ysizes:
            if xsize == 1 or ysize == 1:
                continue
            if xsize != ysize:
                continue
            count += 1
            # print('# {} : [{}] x {}]'.format(count, xsize, ysize))

    print(repr(np.array(xsizes)))
    print(len(xsizes))
    print(4*count + 1)
