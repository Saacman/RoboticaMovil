import numpy as np
import os
def rotx(theta):
    Rx = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    return Rx

def roty(theta):
    Ry = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    return Ry

def rotz(theta):
    Rz = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    return Rz

def transformB2A(euler, trans, pos_b):
    alpha = euler[0]
    beta = euler[1]
    gamma = euler[2]
    AB_R = np.dot(rotx(alpha), np.dot(roty(beta), rotz(gamma)))
    rotated = np.dot(AB_R, pos_b.reshape(3,1))
    return (rotated + np.array(trans).reshape(3,1)).reshape(3)


class GridMap: 
    def __init__(self):
        self.coffset = np.array([0,0])
        self.grid = np.ones((10,10)) * 0.5
        self.path = 'map.txt'
        
    def setPoint(self, point, value):
        point = point[:2]
        p = point + self.coffset
        # Add columns to the left, if needed
        while p[0] < 0:
            #n = abs(p[0])
            # Add 10 columns at once, to avoid using this block too often
            bffr = np.ones((self.grid.shape[0], 10)) * 0.5
            self.grid = np.concatenate((bffr, self.grid), axis=1)
            # Update the offset
            self.coffset[0] = self.coffset[0] + 10
            # Try again
            p = point + self.coffset
        # Add rows under the current array, if needed
        while p[1] < 0:
            bffr = np.ones((10, self.grid.shape[1])) * 0.5
            self.grid = np.concatenate((bffr, self.grid), axis=0)
            # Update the offset
            self.coffset[1] = self.coffset[1] + 10
            # Try again
            p = point + self.coffset
        # Add columns to the Right, if needed
        while p[0] >= self.grid.shape[1]:
            bffr = np.ones((self.grid.shape[0], 10)) * 0.5
            self.grid = np.concatenate((self.grid, bffr), axis=1)
            
        # Add rows above the current array, if needed
        while p[1] >= self.grid.shape[0]:
            bffr = np.ones((10, self.grid.shape[1])) * 0.5
            self.grid = np.concatenate((self.grid, bffr), axis=0)

        # Finally, add the value to the grid
        self.grid[p[1], p[0]] = value
            
    def getGrid(self):
        return np.flipud(self.grid)

    def saveGrid2File(self):
        np.savetxt(self.path, self.grid)