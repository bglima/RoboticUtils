from robutils import *

if __name__ == '__main__':
    # Configuring numpy precision
    np.set_printoptions(precision=4, suppress=True)

    # Creating a rotation of 90 degrees around X axis
    q0 = [1, 0, 0, 0]
    theta = np.pi / 2
    q1 = [np.cos(theta/2), np.sin(theta/2), 0, 0]
    steps = 20

    # Slerping from q0 to q1
    quats = quatSLERP(q0, q1, steps)

    # Seeing grpahical results
    ax = initPlot()

    # Defining first and last vectors from SLERP
    u = np.array([0.7071, 0.7071, 0]).reshape(3, 1)
    v = np.matmul(rotMatrixFromQuat(quats[-1]), u)

    # Protting them beforehand
    plotLine(ax, u, color='magenta', coord=True)
    plotLine(ax, v, color='green', coord=True)

    # For every intermediary quaternion...
    for q in quats[1:-1]:
        # Extract rotation matrix
        r = rotMatrixFromQuat(q)
        # And draw intermediary vector
        plotLine(ax, np.matmul(r, u), color='orange', linestyle='--')

    plt.show()
