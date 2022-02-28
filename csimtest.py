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
#for i in range(16):
#    err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_streaming)


t = time.time()
iter = 0
while (time.time()-t) < 3:
    uRight, uLeft = v2u(0.5, 3.1415/12.0, 0.09, 0.33)
    err = sim.simxSetJointTargetVelocity(clientID, motorL, 1, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, motorR, 0, sim.simx_opmode_streaming)
    smeasure = []
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
        print('Sensor {}. Detection: {}'.format(i+1, state))
        if state:
            print('Coords: {}    Distance: {}'.format(point, smeasure[-1]))
            print('Obj: {}'.format(detectedObj))
    time.sleep(0.001)
    iter = iter + 1

print(iter/(time.time()-t))

while (time.time()-t) < 3.1:
    err = sim.simxSetJointTargetVelocity(clientID, motorL, 0.0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, motorR, 0.0, sim.simx_opmode_streaming)
    time.sleep(0.1)

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
