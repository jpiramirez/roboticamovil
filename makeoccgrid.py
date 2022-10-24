#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Occupancy grid creation using a Pioneer pd3x with ultrasonic sensors.

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2022)
"""


import numpy as np
import time
import math as m
import random
import sys
import matplotlib.pyplot as plt
import os
import sim  # access all the sim elements
from skimage.draw import line

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1/2*(x**2+y**2)
    return R


sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
err, motorR = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(0,16):
    err, s = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_streaming)

ret, carpos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
ret, carrot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((100,100))
    tocc = np.zeros((100,100))
t = time.time()

initt = t
niter = 0
while time.time()-t < 30:
    ret, carpos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)

    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    ret, carrot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

    uread = []
    ustate = []
    upt = []
    for i in range(0,16,4):
       err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
       ret, objpos = sim.simxGetObjectPosition(clientID, detectedObj, -1, sim.simx_opmode_blocking)
       uread.append(np.linalg.norm(point))
       upt.append(point)
       ustate.append(state)
       ret, srot = sim.simxGetObjectQuaternion(clientID, usensor[i], -1, sim.simx_opmode_blocking)
       ret, spos = sim.simxGetObjectPosition(clientID, usensor[i], -1, sim.simx_opmode_blocking)
       R = q2R(srot[0], srot[1], srot[2], srot[3])
       spos = np.array(spos).reshape((3,1))
       if i % 2 != 0:
           continue
       if state == True:

           opos = np.array(point).reshape((3,1))

           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100

           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0
           tocc[yo-1, xo-1] = 1

       else:
           opos = np.array([0,0,1]).reshape((3,1))

           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0

    ul = 1
    ur = 1
    lgains = np.linspace(0,-1,len(upt)//2)
    rgains = np.linspace(-1,0,len(upt)//2)
    for k in range(len(upt)//2):
        if ustate[k]:
            ul = ul + lgains[k]*(1.0 - uread[k])
            ur = ur + rgains[k]*(1.0 - uread[k])
    print('lvel {}   rvel {}'.format(ul, ur))

    errf = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_oneshot)
    errf = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_oneshot)

    niter = niter + 1

print(lgains)
print(rgains)
finalt = time.time()
print('Avg time per iteration ', (finalt-initt)/niter)

plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)
for i in range(10):
    errf = sim.simxSetJointTargetVelocity(clientID, motorL, 0.0, sim.simx_opmode_oneshot)
    errf = sim.simxSetJointTargetVelocity(clientID, motorR, 0.0, sim.simx_opmode_oneshot)
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
