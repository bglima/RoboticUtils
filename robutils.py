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
def quatProd(u1, u2):
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
def quatFromAxisAngle(axis, angle):
    # Defining uuaternion as a numpy array
    u = np.zeros(3)
    # Scalar part from uuaternion
    u0 = np.cos(angle / 2)
    # Vector part
    u = np.sin(angle / 2) * axis
    # Combining both into output uuaternion
    q = [ u0, u[0][0], u[1][0], u[2][0] ]
    return q

""" Performs a SLERP between two 1-by-4 unit quaternions and return all the intermediate quaternions.
    Thus, return will be an array of STEPS quaternios, with first one being q1 and last one being q2.
    The shortest path between q1 and q2 is always choosen. """
def quatSLERP(q0, q1, steps):
    # Array that will be returned
    q_int = np.zeros((steps, 4))
    # Creating a copy of q1
    q1_updated = q1;
    # Let theta be the angle between quaternions
    # Also, let omega be equal to theta / 2
    omega = np.arccos( np.dot(q0, q1) )
    theta = omega*2
    # Checking if theta is larger than pi. If so, chose shortest angle
    if theta > np.pi:
        q1_updated = q1 * (-1)
        theta = 2*np.pi - theta
        omega = theta / 2

    for step in range(0, steps):
        t = float(step) / (steps-1)
        q_int[step] = np.multiply( ( np.sin((1-t)*omega) / np.sin(omega)), q0) + np.multiply( (np.sin(t*omega) / np.sin(omega)), q1)
    return q_int

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

""" Returns a tuple containing a 3-by-1 axis and a scalar angle form a 3-by-3 rotation matrix.
    If angle is 0, there are infinite solutions. So, axis vector will be filled with NaN.
    If angle is PI, there will be two possible solutions for the same rotation (PI and -PI).
    Othewise, there will be only one possible angle and one possible axis.
    References from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
"""
def axisAngleFromRotMatrix( R ):
    angle = np.arccos( (np.trace(R)-1)/2.0 )
    w = np.zeros(3).reshape(3, 1)

    if angle == 0 :          # If angle is zero, fill vector with NaN
        w.fill(np.nan)
    elif angle == math.pi :  # If angle is PI, calculate two solutions (w, angle) and (-w, angle)
        xx = ( R[0][0] + 1 ) / 2.0
        yy = ( R[1][1] + 1 ) / 2.0
        zz = ( R[2][2] + 1 ) / 2.0
        xy = ( R[0][1] + R[1][0] ) / 4.0
        xz = ( R[0][2] + R[2][0] ) / 4.0
        yz = ( R[1][2] + R[2][1] ) / 4.0

        # Checking for bigger diagonal to avoid numerical underflow
        if (xx > yy) and (xx > zz):  # R(0, 0) is the largest diagonal term
            x = math.sqrt(xx)
            y = xy / x
            z = xz / x
        elif ( yy > zz):            # R(1, 1) is the largest diagonal term
            y = math.sqrt(yy)
            x = xy / y
            z = xz / y
        else:                       # R(3, 3) is the largest diagonal term
            z = math.sqrt(zz)
            x = xz / z
            y = yz / z
        # Compose double solution
        w = np.array([ np.array([x, y, z]).reshape(3, 1), np.array([-x, -y, -z]).reshape(3, 1) ])
        angle = np.array([angle, angle])

    else:                    # If angle is neither 0 nor PI, calculate single solution
        w = np.array([R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]]) * 1 / (2 * np.sin(angle))

    return (w, angle)

""" Plot a line between two points given an axis with a specific color"""
def plotLine(ax, p2, p1=np.zeros(3).reshape(3, 1), color='black', label='', coord=False, linestyle='-'):
    p1 = np.array(p1)
    p2 = np.array(p2)
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
def initPlot(max_range=3, width=6, height=6):
    # Define figure
    fig = plt.figure(figsize=(width, height))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
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

    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range*.25, 2*max_range)
    return ax
