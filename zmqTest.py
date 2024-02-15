"""
Test for CoppeliaSim using the Pioneer pd3x robot (ZeroMQ API)

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""

import time
import math as m

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot  = sim.getObject("/PioneerP3DX")

sim.startSimulation()
r = 0.5*0.195
L = 2.0*0.1655

pos = sim.getObjectPosition(robot, -1)
print(f'Posicion inicial robot {pos}')
for k in range(4):
    tstart = sim.getSimulationTime()
    while sim.getSimulationTime()-tstart < 6:
        #ur, ul = v2u(0, (m.pi/2.0)/5.0, r, L)
        ur, ul = v2u(0.5, 0, r, L)
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)
    tstart = sim.getSimulationTime()
    while sim.getSimulationTime()-tstart < 6:
        #ur, ul = v2u(0, (m.pi/2.0)/5.0, r, L)
        ur, ul = v2u(0, m.pi/12.0, r, L)
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)
#sim.pauseSimulation()
#time.sleep(5)
pos = sim.getObjectPosition(robot, -1)
print(f'Posicion final robot {pos}')

sim.stopSimulation()