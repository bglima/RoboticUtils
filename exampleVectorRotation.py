from utils import *

if __name__ == '__main__':
    # Generate a random normalized axis vector
    w = np.random.rand(3, 1)
    w_norm = w / np.linalg.norm(w)

    # Plot lines
    ax = initPlot()
    plotLine(ax, w_norm, color='black')

    plt.show()
