"""
Robot set-point control (regulation) using a Pioneer p3dx from CoppeliaSim

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020-2022)
"""

import numpy as np
import time
import math as m
import sim # access all the sim elements

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)


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

ret, carpos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
ret, carrot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)

# Controller gains (linear and heading)
Kv = 0.3
Kh = 0.8

# xd and yd are the coordinates of the desired setpoint
xd = 1
yd = 1

hd = 0
r = 0.5*0.195
L = 0.311
errp = 1000

while errp > 0.1:
    ret, carpos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
    ret, carrot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angdiff(carrot[2], angd)
    print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

# Uncomment for switched control
   # if errh > 2.0*m.pi/180.0:
   #     v = 0
   #     omega = -Kh*errh
   # else:
   #     v = Kv*err
   #     omega = 0
# Uncomment for continuous control
    v = Kv*errp
    omega = Kh*errh

    ur, ul = v2u(v, omega, r, L)
    errf = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_oneshot)
    errf = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_oneshot)
    #time.sleep(0.1)

for i in range(10):
    errf = sim.simxSetJointTargetVelocity(clientID, motorL, 0, sim.simx_opmode_oneshot)
    errf = sim.simxSetJointTargetVelocity(clientID, motorR, 0, sim.simx_opmode_oneshot)
    time.sleep(0.05)
    
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
