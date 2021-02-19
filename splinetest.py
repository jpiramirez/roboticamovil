"""
Trajectory generation based on b-spline curves

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

ttime = 10
xarr = np.array([0,3,2,3])
yarr = np.array([0,2,1,3])
tarr = np.linspace(0, 10, xarr.shape[0])
  
tnew = np.linspace(0, 10, 100)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
plt.show()