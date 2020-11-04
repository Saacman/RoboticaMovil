import numpy as np

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
    def __init__(self, o_pos):
        self.coffset = (0,0)
        self.grid = np.ones((10,10)) * 0.5
        
    def setPoint(self, point, value):
        p = point + self.coffset
        # Add columns to the left, if needed
        while p[0] < 0:
            nw = np.ones((self.grid.shape[0], )) * 0.5
            self.grid = np.concatenate(())




    