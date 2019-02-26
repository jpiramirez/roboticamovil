import numpy as np
import time
import math as m
import vrep # access all the VREP elements
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

Kv = 0.5;
Kh = 2.5;
xd = 0;
yd = 0;
hd = 0;
r = 0.1;
L = 0.2;
errp = 10;

while errp > 0.1:
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    print(carpos)
    print(carrot)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    print(errp)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angd-carrot[2]

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


    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    time.sleep(0.1)


vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
