"""
Simulating a DDR using an ODE solver

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import frameplot as fp

from scipy.integrate import solve_ivp

def ddr1(t, y):
    r = 0.5*0.195
    L = 0.311
    v = 1.0
    omega = 2.0*np.pi/10.0
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    xdot = v*np.cos(y[2])
    ydot = v*np.sin(y[2])
    thdot = omega
    return np.array([xdot, ydot, thdot])

def ddr2(t, y):
    r = 0.5*0.195
    L = 0.311
    v = 0
    omega = 0
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    xdot = v*np.cos(y[2])
    ydot = v*np.sin(y[2])
    thdot = omega
    return np.array([xdot, ydot, thdot])

def ddr3(t, y):
    r = 0.5*0.195
    L = 0.311
    v = 0
    omega = 0
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    xdot = v*np.cos(y[2])
    ydot = v*np.sin(y[2])
    thdot = omega
    return np.array([xdot, ydot, thdot])

sol1 = solve_ivp(ddr1, [0,10], np.array([0,0,0]), max_step=0.1)
sol2 = solve_ivp(ddr2, [10,13], sol1.y[:,-1], max_step=0.1)
sol3 = solve_ivp(ddr3, [13,23], sol2.y[:,-1], max_step=0.1)

solT = np.concatenate((sol1.y, sol2.y, sol3.y), axis=1)
time = np.concatenate((sol1.t, sol2.t, sol3.t))
print(sol1.t)
print(sol1.status)

fig = plt.figure()
ax = fig.gca()
ax.plot(time, solT[2,:])
#ax.plot(solT[0,:], solT[1,:])

plt.show()