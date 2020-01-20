#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 12 15:00:22 2019

@author: jpiramirez
"""


import numpy as np
import time
import math as m
import random
import matplotlib.pyplot as plt
import os
import vrep # access all the VREP elements
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


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

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
plt.figure(2)
t = time.time()

while time.time()-t < 60:
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)

    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    print('Rows {}   Cols {}'.format(yr, xr))
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angd-carrot[2]

    uread = []
    ustate = []
    upt = []
    for i in range(16):
       err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
       print(detectedObj)
       ret, objpos = vrep.simxGetObjectPosition(clientID, detectedObj, -1, vrep.simx_opmode_blocking)
       print(objpos)
       uread.append(np.linalg.norm(point))
       upt.append(point)
       ustate.append(state)
       ret, srot = vrep.simxGetObjectQuaternion(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       ret, spos = vrep.simxGetObjectPosition(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
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

    v = 0.5
    if v > 0.5:
        v = 0.5

    omega = 0
    if omega > 2.5:
        omega = 2.5
    elif omega < -2.5:
        omega = -2.5

    if ustate[2] == True and uread[2] < 0.8:
        print('Imminent collision at '+str(uread[4]))
        omega = -1.5-0.1*random.random()
        v = 0.1+0.1*random.random()
    if ustate[5] == True and uread[5] < 0.8:
        print('Imminent collision at '+str(uread[4]))
        omega = 1.5+0.1*random.random()
        v = 0.1+0.1*random.random()

    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    time.sleep(0.005)

plt.figure(1)
plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
