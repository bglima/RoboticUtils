# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 14:30:20 2018

@author: brunolima
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math

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

""" m is a 3-by-1 column vector.
    Returns a antissymetric 3-by-3 matrix J(m) that substitues cross product from m by a matrix multiplication.
    Thus, cross product m x n becomes the matrices multiplication J(m)*n """
def J( m ):
    out = np.zeros((3, 3))
    out[0][1] = -m.item(2); out[1][0] =  m.item(2);
    out[0][2] =  m.item(1); out[2][0] = -m.item(1);
    out[1][2] = -m.item(0); out[2][1] =  m.item(0);
    return out

""" Returns a rotation matrix from a given axis and angle. """
def rotMatrixFromAxisAngle(axis, angle):
    # Uses the Rodriges formula, as follows...
    R = np.eye(3) + math.sin(angle)*J(axis) + ( 1-math.cos(angle) )*np.matmul(J(axis), J(axis))
    return R

""" Plot a line between two points given an axis with a specific color"""
def plotLine(ax, p2, p1=np.zeros(3).reshape(3, 1), color='black', label='', coord=False, linestyle='-'):
    ax.plot([p1[0][0], p2[0][0]],
             [p1[1][0], p2[1][0]],
             [p1[2][0], p2[2][0]],
             color=color,
             linestyle=linestyle)
    if label:
        ax.text(p2[0][0], p2[1][0], p2[2][0] , label)
    if coord:
        ax.text(p2[0][0], p2[1][0], p2[2][0] - 0.1 , '({:.3f}, {:.3f}, {:.3f})'.format(p2[0][0], p2[1][0], p2[2][0]))

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
