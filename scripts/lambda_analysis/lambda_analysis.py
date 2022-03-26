import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif":  ["Palatino"],
    "font.size":   18
})

filenames = []


# sense action cost
def function_z(length, ncovered, kLambda):
    return length - kLambda * ncovered + 1e6

def generate_plots():
    # Different surfaces for different values of lambda
    count = 0
    # for kLambda in np.arange(0.0, 10.0+0.5, 0.5):
    for kLambda in [0.0, 1.0, 3.0]:
        # X = length
        # Y = ncovered
        # Z = cost

        x_values = []
        y_values = []
        for length_x in np.arange(1.0, 30.0, 1.0):
            for ncovered_y in np.arange(0, 30, 1):
                x_values.append(length_x)
                y_values.append(ncovered_y)

        x_values = np.array(x_values)
        y_values = np.array(y_values)

        X, Y = np.meshgrid(x_values, y_values)
        Z = function_z(X, Y, kLambda);

        fig, ax = plt.subplots()
        ax.set_xlabel(r'length')
        ax.set_ylabel(r'$n_\mathsf{covered}$')
        ax.set_title(r'$\lambda$ = {}'.format(kLambda))

        c = ax.pcolormesh(X, Y, Z, cmap='RdYlBu_r')
        fig.colorbar(c, ax=ax)

        count += 1
        filename = 'lambda_analysis_imgs/lambda_equals_' + str(int(kLambda)) + '_img.pdf'
        filenames.append(filename)
        # plt.savefig(filename, dpi=300)
        plt.savefig(filename, dpi=300)
        # plt.show()

if __name__=='__main__':
    generate_plots()
