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

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

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
errh = 10

while np.abs(errh) > 0.01:
    smeasure = []
    statev = [False, False]
    for i in range(7,9):
        err = 1
        while err:
            err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
        #print(err)
        #print('Sensor {}. Detection: {}'.format(i, state))
        statev[i-7] = state
        #    print('Coords: {}    Distance: {}'.format(point, smeasure[-1]))
        #    print('Obj: {}'.format(detectedObj))
        #print(state)
    if statev[0] and statev[1]:
        r = 0.5*0.195
        errp = 1000
        ang = (smeasure[0]-smeasure[1])/L
        print('Angle: {}'.format(180*ang/np.pi))
        errh = angdiff(ang, 0)
        v = 0.05
        Kh = 0.5
        omega = Kh*errh
        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)
        errf = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_streaming)
        errf = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_streaming)
    time.sleep(0.001)
    iter = iter + 1

print(iter/(time.time()-t))

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
