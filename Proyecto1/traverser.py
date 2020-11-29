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
# Load the grid, and make it thicc
theMap = GridMap.loadImg("map_v2.png")
grid = theMap.getGrid()
plt.imshow(grid)
plt.show()

bin_grid = np.int8(grid > 0.5)
disk = selem.disk(3)
bin_grid = binary_dilation(bin_grid, disk)
plt.imshow(bin_grid)
plt.show()

start = (89,70) #Initial pos (y,x)
end = (17, 11)
path = astar(bin_grid, start, end, allow_diagonal_movement=True)
path = np.array(path)
grid[path[:,0], path[:,1]] = 0.5

plt.imshow(grid)
plt.show()

#<-----------------------------------Control----------------------------------------->

pointsx = path_transformed[:,0]
pointsy = path_transformed[:,1]

step = 0
errp = 10
achieved = 0
avoid = False

while step < len(pointsx):
    # Traverse the path
    step = step + 1 if errp < 0.1 else step
    # Check obstacles or go to next point in path
    avoid, ulb, urb = pc.braitenberg(clientID, usensor)
    errp, ulc, urc, pos, rot = pc.continuosControl(clientID, robot, (pointsx[step], pointsy[step]))

    ul = ulb if avoid else ulc
    ur = urb if avoid else urc
    

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)


# The End
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
