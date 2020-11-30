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
"""
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
"""
#<----------------------------------A* Algorithm---------------------------------------->

# Load the grid, and make it thicc
theMap = GridMap.loadImg("map.png")
grid = theMap.getGrid()

bin_grid = np.int8(grid > 0.5)
disk = selem.disk(3)
bin_grid = binary_dilation(bin_grid, disk)

#err, pos0 = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
pos0 = [ 1.10356879, -3.55007982]
pos0 = ((pos0 - theMap.ipos) / theMap.tsize).astype(int) + theMap.coffset
pos0[1] = -pos0[1] + 110
start = (70,89)
print((70-60)*0.1+pos0[0])
print((89-20)*0.1+pos0[1])
end = (11, 17)
path = astar(bin_grid, start, end, allow_diagonal_movement=True)
path = np.array(path)
grid[path[:,1], path[:,0]] = 0.5
plt.imshow(grid)
plt.show()
print(theMap.coffset)
#<-----------------------------------Control----------------------------------------->
path = path.astype(float)
path[:,1] = -path[:,1] + len(grid)
for i in range(len(path)):
    path[i,:] = (path[i,:] - theMap.coffset) * theMap.tsize + theMap.ipos


"""
pointsx = path[:,0]
pointsy = path[:,1]

step = 0
errp = 10
avoid = False

while step < len(pointsx):
    # Traverse the path
    step = step + 2 if errp < 0.2 else step
    # Check obstacles or go to next point in path
    avoid, ulb, urb = pc.braitenberg(clientID, usensor)
    errp, ulc, urc, pos, rot = pc.continuosControl(clientID, robot, (pointsx[step], pointsy[step]))

    ul = ulb if avoid else ulc
    ur = urb if avoid else urc
    

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)

#The End
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
"""