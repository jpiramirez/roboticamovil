import time
import math as m

from zmqRemoteApi import RemoteAPIClient

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

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject("/PioneerP3DX/leftMotor")
motorR=sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

sim.startSimulation()

xd = -2
yd = 0

Kv = 0.3
Kh = 0.8
r = 0.5*0.195
L = 0.311

errp = 1000

while errp > 0.1:
    carpos = sim.getObjectPosition(robot, -1)
    carrot = sim.getObjectOrientation(robot, -1)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angdiff(carrot[2], angd)
    print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

    v = Kv*errp
    omega = Kh*errh

    ur, ul = v2u(v, omega, r, L)
    errf = sim.setJointTargetVelocity(motorL, ul)
    errf = sim.setJointTargetVelocity(motorR, ur)
    print(errf)

time.sleep(1)

sim.stopSimulation()