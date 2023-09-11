"""
Test for CoppeliaSim using the Pioneer pd3x robot (ZeroMQ API)

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""

import time
import numpy as np
import math as m

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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
    R[2,2] = 1-2*(x**2+y**2)
    return R

client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

sensor = sim.getObject("/sensor")
sphere0 = sim.getObject("/Sphere[0]")
sphere1 = sim.getObject("/Sphere[1]")

sim.startSimulation()

while sim.getSimulationTime() < 1:
    result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(sensor)
    print(f'res={result}  dist={distance} point={detectedPoint} handle={detectedObjectHandle} normal={detectedSurfaceNormalVector}')
    t = sim.getObjectPosition(sensor, sim.handle_world)
    q = sim.getObjectQuaternion(sensor, sim.handle_world)
    R = q2R(q[0], q[1], q[2], q[3])
    print(f'translation={t}   quaternion={q}')
    print('Rotation matrix:')
    print(R)
    pw = R @ np.array(detectedPoint) + np.array(t)
    pw_real = R @ np.array([0,0,distance]) + np.array(t)
    print(f'ideal point in world coordinates : {pw}')
    print(f' real point in world coordinates : {pw_real}')
    sim.setObjectPosition(sphere0, sim.handle_world, pw.tolist())
    sim.setObjectPosition(sphere1, sim.handle_world, pw_real.tolist())

sim.pauseSimulation()
time.sleep(5)

sim.stopSimulation()