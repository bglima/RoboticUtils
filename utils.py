# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 14:30:20 2018

@author: brunolima
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

""" Performs multiplication from quaternions q1 and q2, returning the quaternion result q. """
def quatProd(q1, q2):
    # All quaternions q, q1 and q2 are represented as 1x4 row vectors
    q = np.zeros(4)
    # Separate scalar from vector part from input quaternions
    u0 = q1[0]
    v0 = q2[0]
    u = q1[1:]
    v = q2[1:]
    # Defining scalar and vector part from new quaternion
    q[0] = u0*v0 - np.dot(u, v)
    q[1:] = u0*v + v0*u + np.cross(u, v)
    return q

""" Plot a line between two points given an axis with a specific color"""
def plotLine(ax, p2, p1=np.zeros(3).reshape(3, 1), color='black', label='', coord=True):
    ax.plot([p1[0][0], p2[0][0]],
             [p1[1][0], p2[1][0]],
             [p1[2][0], p2[2][0]],
             color=color)
    if label:
        ax.text(p2[0][0], p2[1][0], p2[2][0] , label)
    elif coord:
        ax.text(p2[0][0], p2[1][0], p2[2][0] , '({:.2f}, {:.2f}, {:.2f})'.format(p2[0][0], p2[1][0], p2[2][0]))
""" Init and setup axis. Needed before plotting vectors. """
def initPlot():
    # Define figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Defining origin and unit vectors
    origin = np.array([0, 0, 0]).reshape(3, 1)  # Set a 3-by-1 column vector
    x_axis = np.array([1, 0, 0]).reshape(3, 1)
    y_axis = np.array([0, 1, 0]).reshape(3, 1)
    z_axis = np.array([0, 0, 1]).reshape(3, 1)

    # Defining initial view angle
    ax.view_init(30, 90)

    # Hide axes ticks
    plt.axis('off')

    # Plot axis lines
    plotLine(ax, x_axis, origin, 'red', 'X Axis')
    plotLine(ax, y_axis, origin, 'green', 'Y Axis')
    plotLine(ax, z_axis, origin, 'blue', 'Z Axis')
    return ax

if __name__ == '__main__':
    # Generate a random normalized axis vector
    w = np.random.rand(3, 1)
    w_norm = w / np.linalg.norm(w)

    # Plot lines
    ax = initPlot()
    plotLine(ax, w_norm, color='black')
    plt.show()
