import numpy as np
import time
import math as m
import sys
import vrep # access all the VREP elements
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)


#err, val = vrep.simxGetFloatSignal(clientID, 'accelerometerX', vrep.simx_opmode_streaming)

r = 0.1
L = 0.331
P = m.pi*2.0*r
dist = 1.0
v = 0.1
omega = 0
ur = v/r + L*omega/(2*r)
ul = v/r - L*omega/(2*r)
t1 = dist/v
print('Computed velocity for right motor: '+str(ur))
print('Computed velocity for left motor: '+str(ul))
print('Moving forward '+str(t1)+' seconds to traverse '+str(dist)+' m')

t = time.time()
err, jvel = vrep.simxGetObjectFloatParameter(clientID,
                motorL,
                2012, # ID for angular velocity of the joint
                vrep.simx_opmode_streaming)
while (time.time()-t) < t1:
    #err, val = vrep.simxGetFloatSignal(clientID, 'accelerometerX', vrep.simx_opmode_buffer)
    #print(val)
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    err, jvel = vrep.simxGetObjectFloatParameter(clientID, motorL, 2012, vrep.simx_opmode_blocking)
    smeasure = []
    print(str(time.time()-t)+' '+str(jvel))
    time.sleep(0.1)

v = 0
angle = m.pi/2
omega = m.pi/8
ur = v/r + L*omega/(2*r)
ul = v/r - L*omega/(2*r)
t2 = angle/omega
print(ur)
print(ul)

t = time.time()
while (time.time()-t) < t2:
    #err, val = vrep.simxGetFloatSignal(clientID, 'accelerometerX', vrep.simx_opmode_buffer)
    #print(val)
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    smeasure = []
    time.sleep(0.1)


vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)


