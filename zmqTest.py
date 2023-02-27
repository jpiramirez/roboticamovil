"""
Test for CoppeliaSim using the Pioneer pd3x robot (ZeroMQ API)

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""

import time
import math as m

from zmqRemoteApi import RemoteAPIClient

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

while sim.getSimulationTime() < 5:
    ur, ul = v2u(0, 2.0*m.pi/15.0, r, L)
    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

sim.pauseSimulation()
time.sleep(1)

sim.stopSimulation()