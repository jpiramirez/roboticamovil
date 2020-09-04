"""
Helper functions to plot frames of reference in 3D

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def frame(ax, t, R, fcolor='none', flabel='none'):
    p = np.tile(t, (1,3)) + R
    if fcolor=='none':
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,0], R[1,0], R[2,0], color='r')
        ax.text(p[0,0], p[1,0], p[2,0], 'X', color='r')
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,1], R[1,1], R[2,1], color='g')
        ax.text(p[0,1], p[1,1], p[2,1], 'Y', color='g')
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,2], R[1,2], R[2,2], color='b')
        ax.text(p[0,2], p[1,2], p[2,2], 'Z', color='b')
        if flabel != 'none':
            ax.text(t[0,0]-.2, t[1,0]-.2, t[2,0]-.2, flabel, color='k')
    else:
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,0], R[1,0], R[2,0], color=fcolor)
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,1], R[1,1], R[2,1], color=fcolor)
        ax.quiver(t[0,0], t[1,0], t[2,0], R[0,2], R[1,2], R[2,2], color=fcolor)
        ax.text(p[0,0], p[1,0], p[2,0], 'X', color=fcolor)
        ax.text(p[0,1], p[1,1], p[2,1], 'Y', color=fcolor)
        ax.text(p[0,2], p[1,2], p[2,2], 'Z', color=fcolor)
        if flabel != 'none':
            ax.text(t[0,0]-.2, t[1,0]-.2, t[2,0]-.2, flabel, color=fcolor)

def rotx(theta):
    Rx = np.matrix([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    return Rx

def roty(theta):
    Ry = np.matrix([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    return Ry

def rotz(theta):
    Rz = np.matrix([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    return Rz

def skew(v):
    return np.matrix([[0, -v[2,0], v[1,0]],[v[2,0],0,-v[0,0]],[-v[1,0], v[0,0], 0]])

def vex(S):
    return np.matrix([[S[2,1],S[0,2],S[1,0]]]).T

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    t = np.matrix([[1,2,3]]).T
    frame(ax, np.zeros((3,1)), np.eye(3), fcolor='b', flabel='{w}')
    frame(ax, t, roty(np.pi/5)*rotx(np.pi/4.0), flabel='{R}')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    print(vex(skew(t)))
    plt.show()