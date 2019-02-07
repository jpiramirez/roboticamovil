import numpy as np
import time
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


t = time.time()

while (time.time()-t) < 10:
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, 1.0, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, 0.0, vrep.simx_opmode_streaming)
    smeasure = []
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
    print(smeasure)        
    time.sleep(0.1)

while (time.time()-t) < 20:
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, 0.0, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, 1.0, vrep.simx_opmode_streaming)
    time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
