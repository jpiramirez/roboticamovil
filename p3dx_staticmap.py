"""
Demonstration of a Python-CoppeliaSim connection using a Pioneer 3dx simulation

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2022)
"""
import numpy as np
import time
import math as m
import sys
import sim
import matplotlib.pyplot as plt

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

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors
err, motorL = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
err, motorR = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(16):
    err, s = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_streaming)

err = 1
err, spos = sim.simxGetObjectPosition(clientID, usensor[8], usensor[7], sim.simx_opmode_streaming)

while err:
    err, spos = sim.simxGetObjectPosition(clientID, usensor[8], usensor[7], sim.simx_opmode_buffer)
L = np.linalg.norm(np.array(spos))
print(L)

t = time.time()
iter = 0
while (time.time()-t) < 1:
    smeasure = []
    statev = []
    xp = []
    yp = []
    for i in range(16):
        err = 1
        while err:
            err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
        #print(err)
        #print('Sensor {}. Detection: {}'.format(i, state))
        statev.append(state)
        #    print('Coords: {}    Distance: {}'.format(point, smeasure[-1]))
        #    print('Obj: {}'.format(detectedObj))
        #print(state)
        if state:
            err = 1
            err, spos = sim.simxGetObjectPosition(clientID, usensor[i], -1, sim.simx_opmode_streaming)
            while err:
                err, spos = sim.simxGetObjectPosition(clientID, usensor[i], -1, sim.simx_opmode_buffer)
            err = 1
            err, squat = sim.simxGetObjectQuaternion(clientID, usensor[i], -1, sim.simx_opmode_streaming)
            while err:
                err, squat = sim.simxGetObjectQuaternion(clientID, usensor[i], -1, sim.simx_opmode_buffer)
            R = q2R(squat[0], squat[1], squat[2], squat[3])
            spos = np.array(spos).reshape((3,1))
            p = np.array([0,0,smeasure[i]]).reshape((3,1))
            wpos = np.matmul(R,p)+spos
            wposl = wpos.tolist()
            xp.append(wposl[0][0])
            yp.append(wposl[1][0])
    time.sleep(0.001)
    iter = iter + 1
    
print(xp)
print(yp)

plt.plot(xp, yp, '.')
plt.show()

print(iter/(time.time()-t))

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
