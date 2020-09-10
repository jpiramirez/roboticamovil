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

def ddr(t, y):
    r = 0.5*0.195
    L = 0.311
    ur = 1.0
    ul = 0.0
    v = 0.5*r*(ur+ul)
    omega = (r/L)*(ur-ul)
    xdot = v*np.cos(y[2])
    ydot = v*np.sin(y[2])
    thdot = omega
    return np.array([xdot, ydot, thdot])

sol = solve_ivp(ddr, [0,10], np.array([0,0,0]), max_step=0.1)
print(sol.t)
print(sol.status)

fig = plt.figure()
ax = fig.gca()
ax.plot(sol.y[0,:], sol.y[1,:])
#ax.plot(sol.t, sol.y[2,:])
plt.show()