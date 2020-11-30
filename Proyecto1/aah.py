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

print(path)
trans_path = np.empty_like(path).astype(float)
offset = occmap.coffset[::-1]
for i in range(len(path)):
    trans_path[i,:] = (path[i,:] - offset) * occmap.tsize - occmap.ipos

print(trans_path)

grid[path[:,0], path[:,1]] = 0.8
plt.imshow(np.flipud(grid))
plt.show()

"""


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