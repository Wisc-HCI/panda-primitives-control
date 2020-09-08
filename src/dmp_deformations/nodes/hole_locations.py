#!/usr/bin/env python

""" Convert STL of desired surface to a BSplineSurface
 using the BSplinesurface calls from the other library
 Created: 07/10/2020
"""

__author__ = "Mike Hagenow"

import trimesh
import trimesh.sample as sample
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize, Bounds
from scipy.spatial import ConvexHull
from PyBSpline import BSplineSurface
import time

def main():
    stl_file = '/home/mike/Desktop/cowling_4_surface.STL'
    mesh = trimesh.load(stl_file)

    holes_cowling = []
    holes_cowling.append(np.array([-0.090621, -0.0753493, .1282883]))
    holes_cowling.append(np.array([0.0013947, -0.0944305, .1282883]))
    holes_cowling.append(np.array([0.0972435, -0.0953528, .1282883]))

    holes_holder = []
    holes_holder.append(np.array([-0.0129, 0.0125, .014605]))
    holes_holder.append(np.array([0.0125, 0.0125, .014605]))
    holes_holder.append(np.array([0.0379, 0.0125, .014605]))


    # Apply a rotation and a translation to all of the points
    R = np.array([[0.99978826, 0.00928849, -0.01836173],
                  [-0.00907831, 0.9998927, 0.01149706],
                  [0.01846655, -0.01132793, 0.9997653]])


    t = np.array([0.38238362, -0.29635461, 0.012553])

    t_addtl_cowling = np.matmul(R,np.array([0.025*5, 0.025*13, 0.0]).reshape((3,1))).reshape((3,))

    t_addtl_holder = np.matmul(R, np.array([0.025 * 5, 0.025 * 20, 0.0]).reshape((3, 1))).reshape((3,))

    t_cowling = t+t_addtl_cowling
    t_holder = t+ t_addtl_holder

    for ii in range(0, len(holes_cowling)):
        holes_cowling[ii] = (np.matmul(R, holes_cowling[ii].reshape((3, 1))) + t_cowling.reshape((3, 1))).reshape(3, )

    for ii in range(0, len(holes_holder)):
        holes_holder[ii] = (np.matmul(R, holes_holder[ii].reshape((3, 1))) + t_holder.reshape((3, 1))).reshape(
            3, )


    print("HOLES COWLING")
    print(holes_cowling)
    print("HOLES HOLDER")
    print(holes_holder)



if __name__ == "__main__":
    main()




