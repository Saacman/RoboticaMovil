import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
import pathcontrol as pc

#<---------------------------------Initialization--------------------------------------->
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


#<-----------------------------------Control----------------------------------------->
goals = [(1.5, 1.5), (1.5, -1.5), (-1.5, -1.5), (-1.5, 1.5), (0,0)]


for goal in goals:
    errp = 10
    while errp > 0.3:
        avoid, ulb, urb = pc.braitenberg(clientID, usensor)
        errp, ulc, urc = pc.continuosControl(clientID, robot, goal)
        ul = ulb if avoid else ulc
        ur = urb if avoid else urc

        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)


# The End
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
