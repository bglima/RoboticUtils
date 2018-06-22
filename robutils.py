# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 14:30:20 2018

@author: brunolima
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math

""" m is a 3-by-1 column vector.
    Returns a antissymetric 3-by-3 matrix J(m) that substitues cross product from m by a matrix multiplication.
    Thu0, cross product m x n becomes the matrices multiplication J(m)*n """
def J( m ):
    out = np.zeros((3, 3))
    out[0][1] = -m.item(2); out[1][0] =  m.item(2);
    out[0][2] =  m.item(1); out[2][0] = -m.item(1);
    out[1][2] = -m.item(0); out[2][1] =  m.item(0);
    return out

""" Performs multiplication from 1-by-3 uuaternions u1 and u2, returning the uuaternion result u. """
def uuatProd(u1, u2):
    # All uuaternions u, u1 and u2 are represented as 1x4 row vectors
    u = np.zeros(4)
    # Separate scalar from vector part from input uuaternions
    u0 = u1[0]
    v0 = u2[0]
    u = u1[1:]
    v = u2[1:]
    # Defining scalar and vector part from new uuaternion
    u[0] = u0*v0 - np.dot(u, v)
    u[1:] = u0*v + v0*u + np.cross(u, v)
    return u

""" Returns a 1-by-4 uuaternion from a given 3-by-1 axis and a scalar angle. """
def uuatFromAxisAngle(axis, angle):
    # Defining uuaternion as a numpy array
    u = np.zeros(4)
    # Scalar part from uuaternion
    u0 = np.cos(angle / 2)
    # Vector part
    u = np.sin(angle / 2) * axis
    # Combining both into output uuaternion
    u[:] = [ u0, u[0][0], u[1][0], u[2][0] ]
    return u

""" Returns a 3-by-3 rotation matrix from a given 3-by-1 axis and a scalar angle. """
def rotMatrixFromAxisAngle(axis, angle):
    # u0es the Rodriges formula, as follows...
    R = np.eye(3) + math.sin(angle)*J(axis) + ( 1-math.cos(angle) )*np.matmul(J(axis), J(axis))
    return R

""" Returns a 3-by-3 rotation matrix from a given 1-by-4 uuaternion. """
def rotMatrixFromQuat(u):
    # Splitting uuaternion components
    u0, ux, uy, uz = u

    # Defining rotation matrix
    R = np.array([
     [1-2*uy**2-2*uz**2, 2*ux*uy-2*uz*u0, 2*ux*uz+2*uy*u0],
     [2*ux*uy+2*uz*u0, 1-2*ux**2-2*uz**2, 2*uy*uz-2*ux*u0],
     [2*ux*uz-2*uy*u0, 2*uy*uz+2*ux*u0, 1-2*ux**2-2*uy**2]
    ])
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
