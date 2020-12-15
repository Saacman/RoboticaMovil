"""
Proyecto Final
"""
import numpy as np
import time
import sys
import sim as vrep # access all the VREP elements
import pickle
import os
import matplotlib.pyplot as plt
import flightplanning as fp
#<---------------------------------Initialization--------------------------------------->
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Assigning handle to drone target
err, target = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_blocking)
err, drone = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_blocking)
err, sphere = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)

# Load the grid
pkl_file = open('inflated.pkl', 'rb')
grid = pickle.load(pkl_file)
pkl_file.close()
csize = 0.4
pos0 = np.array((-7.5, -7.5, 0))

# Get the starting position from the robot
err, init = vrep.simxGetObjectPosition(clientID, drone, -1, vrep.simx_opmode_blocking)
init = np.array(init)

# Get the goal position
err, end = vrep.simxGetObjectPosition(clientID, sphere, -1, vrep.simx_opmode_blocking)
end = np.array(end)

# Set the path parameters
start = ((init - pos0) / csize).astype(int)
goal = ((end - pos0) / csize).astype(int)
maxp = 4.0

# Find the path
heuristic = fp.getHeuristic(grid,goal)
path = fp.search(start, goal, grid, heuristic, maxp)
path = np.array(path)

# Plot the path
grid[path[:,0], path[:,1], path[:,2]] = 1
bingrid = grid > 0
colors = np.empty(grid.shape, dtype=object)
colors[bingrid] = 'blue'
colors[path[:,0], path[:,1], path[:,2]] = 'green'

bingrid = grid > 0
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(bingrid, facecolors=colors)
plt.show()

globalpath =  np.empty_like(path, dtype=float)
for i in range(len(path)):
    globalpath[i,:] = (path[i,:] * csize) + pos0


for step in globalpath:
    vrep.simxSetObjectPosition(clientID, target, -1, step, vrep.simx_opmode_streaming)
    time.sleep(2)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)