from robutils import *

if __name__ == '__main__':
    # Configuring numpy precision
    np.set_printoptions(precision=4, suppress=True)

    # Generate a random normalized axis vector
    w = np.random.rand(3, 1)
    w = w / np.linalg.norm(w)

    # Generate a fixed angle (easier to debug)
    theta = math.pi / 2

    # Generate a vector that will be rotated
    v = np.random.rand(3, 1)

    # Find rotation matrix from previous axis and angle
    r = rotMatrixFromAxisAngle(w, theta)

    # Rotate v twiceby 90 degrees
    v1 = np.matmul(r, v)
    v2 = np.matmul(r, v1)

    # Plot everything
    ax = initPlot()
    plotLine(ax, w, color='black', linestyle='--')
    plotLine(ax, v, color='magenta', coord=True)
    plotLine(ax, v1, color='cyan')
    plotLine(ax, v2, color='yellow')

    plt.show()
