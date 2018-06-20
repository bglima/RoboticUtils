# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 14:30:20 2018

@author: brunolima
"""

import numpy as np

### Performs multiplication from quaternions q1 and q2, returning the quaternion result q.
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

if __name__ == '__main__':
    q1 = np.random.random(4)
    q1 = q1 / np.linalg.norm(q1)
    
    q2 = np.random.random(4)
    q2 = q2 / np.linalg.norm(q2)
    
    q = quatProd(q1, q2)
    np.linalg.norm(q)