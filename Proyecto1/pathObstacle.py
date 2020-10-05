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
errgoal = 10
# goals = np.array([[-7.5, -7.5], [-6.5,  0.5], [-5.5, -3.5], [-4.5, 1.5], [-0.5, -6.5], [ 1.5, -4.5],
#  [ 2.5, -0.5], [ 3.5, -1.5], [ 6.5, -4.5], [ 9.5, -7.5], [11.5, -4.5]])
goals = np.array([[-6.5,  0.5], [-5.5, -3.5], [-4.5, 1.5], [-0.5, -6.5], [ 1.5, -4.5],
 [ 2.5, -0.5], [ 3.5, -1.5], [ 6.5, -4.5], [ 9.5, -7.5], [11.5, -4.5]])
path = pc.splinePath(goals[:,0], goals[:,1])
points = np.linspace(min(goals[:,0]), max(goals[:,0]), num=100, endpoint=True)

for goal in goals:
    while errgoal > 0.3:
        
        avoid, ul, ur = pc.braitenberg(clientID, usensor)

        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    # Remove reached goals
    goals = np.delete(goals, 0, axis=0)


vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
