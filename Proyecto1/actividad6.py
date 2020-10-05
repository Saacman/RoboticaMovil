
import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
import pathcontrol as pc

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
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
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(
        clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)



# Controller gains (linear and heading)
Kv = 0.5
Kh = 5# 2.5 Increase this weight for sharper turns



goals = [(1.5, 1.5), (1.5, -1.5), (-1.5, -1.5), (-1.5, 1.5)]
errp=10
for goal in goals:
    # xd and yd are the coordinates of the desired setpoint
    xd = goal[0]
    yd = goal[1]
    hd = 0
    r = 0.5*0.195
    L = 0.311
    errp = 10

    while errp > 0.5:
        ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
        ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = angdiff(carrot[2], angd)
        print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

    # Continuous control
        v = Kv*errp
        omega = Kh*errh

        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)
        errp, ul, ur = pc.continuosControl(carpos, carrot, goal)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    #time.sleep(0.1)

for i in range(100):
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
    #time.sleep(0.1)
    
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
