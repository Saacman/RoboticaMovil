"""
Proyecto Final
"""
import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
import pathcontrol as pc
from mapping import transformB2A
import pickle
import os
import matplotlib.pyplot as plt

#<---------------------------------Initialization--------------------------------------->
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1, 17):
    err, s = vrep.simxGetObjectHandle(
        clientID, 'ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(
        clientID, usensor[i], vrep.simx_opmode_streaming)


# Initialize or load the grid
pos0 = np.array((-7.5, -7.5, 0))
csize = 0.4
occg = "occupancy-grid.pkl"
if os.path.exists(occg):
    print("Loading map...")
    pkl_file = open(occg, 'rb')
    grid = pickle.load(pkl_file)
    pkl_file.close()
else:
    print("Creating map...")
    grid = np.zeros((38,38,25), dtype=np.int8)

t = time.time()
while time.time()-t < 60*1:
    for i in range(len(usensor)):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(
            clientID, usensor[i], vrep.simx_opmode_oneshot_wait)
        sensor_pos = vrep.simxGetObjectPosition(
            clientID, usensor[i], -1, vrep.simx_opmode_oneshot_wait)
        sensor_orien = vrep.simxGetObjectOrientation(
            clientID, usensor[i], -1, vrep.simx_opmode_oneshot_wait)
        
        if state:
            global_pos = transformB2A(sensor_orien[1], sensor_pos[1], np.array(point))
            grid_pos = ((global_pos - pos0) / csize).astype(int)
            grid_pos[0] = grid_pos[0] if grid_pos[0] < 38 else 37
            grid_pos[1] = grid_pos[1] if grid_pos[1] < 38 else 37
            grid_pos[2] = grid_pos[2] if grid_pos[2] < 25 else 24
            grid[grid_pos[0], grid_pos[1], grid_pos[2]] = 4
        
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

file = open(occg, "wb")
pickle.dump(grid, file)
file.close()

#Plot the grid
bingrid = grid > 0
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(bingrid)
plt.show()

