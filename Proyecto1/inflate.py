import matplotlib.pyplot as plt
import numpy as np
import os
import pickle


occg = "occupancy-grid.pkl"
if os.path.exists(occg):
    print("Loading map...")
    pkl_file = open(occg, 'rb')
    grid = pickle.load(pkl_file)
    pkl_file.close()

voxel = grid > 0
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxel)
plt.show()

xdim=grid.shape[0]
ydim=grid.shape[1]
zdim=grid.shape[2]

# Inflate
inflated = np.zeros((xdim,ydim,zdim), dtype=np.int8)
for i in range(xdim):
    for j in range(ydim):
        for k in range(zdim):
            if grid[i,j,k] > 0:
                inflated[i,j,k]=4
                inflated[i+1,j,k]=4
                inflated[i-1,j,k]=4
                inflated[i,j+1,k]=4
                inflated[i,j-1,k]=4
                inflated[i,j,k+1]=4
                inflated[i,j,k-1]=4

bingrid = inflated > 0
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(bingrid)
plt.show()

file = open("inflated.pkl", "wb")
pickle.dump(inflated, file)
file.close()