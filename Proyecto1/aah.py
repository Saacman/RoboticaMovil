import matplotlib.pyplot as plt
import numpy as np
import time
import math as m
import sys
import sim as vrep  # access all the VREP elements
from mapping import transformB2A, GridMap
from astarmod import astar
import pathcontrol as pc
from skimage.draw import line
from skimage.morphology import selem, binary_dilation
import os

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

#<----------------------------------A* Algorithm---------------------------------------->

# Load the grid
occmap = GridMap.loadImg("rescue.png")
grid = np.array(occmap.grid, copy=True)

# Dilate the obs
bin_grid = np.int8(grid > 0.5)
disk = selem.disk(4)
obstacles = binary_dilation(bin_grid, disk)

start = (20, 70)
end = (90, 30)

path = astar(obstacles, start, end, allow_diagonal_movement=True)
path = np.array(path)

# Transform the obtained path
trans_path = np.empty_like(path).astype(float)
offset = occmap.coffset[::-1]
trasl = occmap.ipos[::-1]
for i in range(len(path)):
    trans_path[i,:] = (path[i,:] - offset) * occmap.tsize + trasl

#<-----------------------------------Control----------------------------------------->
pointsx = trans_path[:,1]
pointsy = trans_path[:,0]
step = 0
errp = 10
avoid = False

while step < len(pointsx)-1:
    # Traverse the path
    step = step + 1 if errp < 0.2 else step
    # Get the motors speed
    errp, ulc, urc, pos, rot = pc.continuosControl(clientID, robot, (pointsx[step], pointsy[step]))

    ul = ulc
    ur = urc

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)

#The End
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
