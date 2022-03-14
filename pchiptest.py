"""
Trajectory generation based on PCHIP interpolant

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2022)
"""

import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

ttime = 10
xarr = np.random.randint(0, 4, (5,))
yarr = np.random.randint(0, 4, (5,))
tarr = np.linspace(0, 10, xarr.shape[0])
  
tnew = np.linspace(0, 10, 200)

#The next two commented lines show how to call the PCHIP interpolator
#in a simplified manner:
#xnew = spi.pchip_interpolate(tarr, xarr, tnew)
#ynew = spi.pchip_interpolate(tarr, yarr, tnew)

pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

xnew = pcix(tnew)
ynew = pciy(tnew)

pcixdot = pcix.derivative()
pciydot = pciy.derivative()
xdot = pcixdot(tnew)
ydot = pciydot(tnew)

plt.figure(1)
plt.plot(xnew[0], ynew[0], 's')
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
plt.title('Path')
plt.show()

plt.figure(2)
plt.plot(tnew, xnew, 'b', label='x')
plt.plot(tnew, ynew, 'r', label='y')
plt.plot(tnew, xdot, 'c', label='xdot')
plt.plot(tnew, ydot, 'm', label='ydot')
plt.plot(tarr, xarr, '.')
plt.plot(tarr, yarr, '.')
plt.legend()
plt.title('Position and velocity over time')
plt.show()