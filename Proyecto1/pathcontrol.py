from scipy.interpolate import interp1d
import numpy as np
import time
import matplotlib.pyplot as plt
import math as m
import sim as vrep
#<---------------------------------Braitenberg--------------------------------------->

noDetectDist = 0.5
maxDist = 0.2
detectW = np.zeros(16)
braitenbergL = np.array([-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
braitenbergR = np.array([-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
v0 = 4
#<-----------------------------------Control----------------------------------------->
# Controller gains (linear and heading)
Kv = 0.5
Kh = 2.5
hd = 0
r = 0.5*0.195
L = 0.311
#<----------------------------------Functions---------------------------------------->

def braitenberg(clientID, usensor):
    """
    Braitenberg algorithm for the front sensors of the pioneer 3dx
    """
    for i in range(len(usensor)):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        distance = np.linalg.norm(point)
        # if a detection occurs
        if state and (distance < noDetectDist): # don't care about distant objects
            distance = max(distance, maxDist) 
            detectW[i] = 1 - ((distance - maxDist) / (noDetectDist - maxDist)) # Normalize the weight
        else:
            detectW[i] = 0
    dL = np.sum(braitenbergL * detectW)
    dR = np.sum(braitenbergR * detectW)
    vLeft = v0 + dL
    vRight = v0 + dR
    avoid = True if (abs(dL) + abs(dR)) else False
    return avoid, vLeft, vRight

def transfB2A(rotAngle, B_ACoords):
    """
    Returns the 2d transformation matrix {B} to {A}
    """
    T = np.array([[np.cos(rotAngle) , np.sin(rotAngle), float(B_ACoords[0])],
                  [-np.sin(rotAngle), np.cos(rotAngle), float(B_ACoords[1])],
                  [                0,                0,           1]])
    return T

def splinePath(x, y):
    """
    Generate a function of the path that matches the goals
    """
    f = interp1d(x, y, kind='cubic')
    return f

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def continuosControl(clientID, robot, goal):
    """
    Provide control for the piooner 3dx given a goal
    """
    xd = goal[0]
    yd = goal[1]
    ret, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, rot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)

    errp = m.sqrt((xd-pos[0])**2 + (yd-pos[1])**2)
    angd = m.atan2(yd-pos[1], xd-pos[0])
    errh = angdiff(rot[2], angd)
    v = Kv*errp
    omega = Kh*errh
    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    return errp, ul, ur, pos, rot

def distance2p(a, b):
    """
    Returns the distance between 2 given points in R2
    """
    d = m.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    return d

if __name__ == "__main__":
    np.random.seed(234)
    #rng = np.random.default_rng()
    xx = np.array([0, 1, 2, 3, 7, 9, 10, 11, 14, 17, 19])
    #xx = np.sort(np.random.randint(1, 15, 10))
    #yy = np.random.randint(0, 15, 10)
    yy = np.array([0, 8, 4, 9, 1, 3, 7, 6, 3, 0, 3])
    func = splinePath(xx, yy)
    xnew = np.linspace(0, max(xx), num=100, endpoint=True)
    plt.plot(xx, yy, 'o', xnew,func(xnew))
    plt.show()
    goals = [(0,0), (1,8), (2,4), (3,9), (7,1), (9,3), (10,7), (11,6), (14,3), (17,0), (19,3)]
    goals = np.array(goals)
    
    T = transfB2A(0, (-7.5, -7.5))
    new = []
    for i in goals:
        N = np.array([[i[0]],
                   [i[1]],
                   [  1  ]])
        hold = np.dot(T,N)
        new.append((hold[0][0],hold[1][0]))
    new = np.array(new)
    print(new[:,0])
    func = splinePath(new[:,0], new[:,1])
    xnew = np.linspace(min(new[:,0]), max(new[:,0]), num=100, endpoint=True)
    plt.plot(new[:,0], new[:,1], 'o', xnew,func(xnew))
    plt.show()
    print(new)

    g = splinePath(np.array([0,2]), np.array([0,4]))
    xnew = np.linspace(0, 2, num=100, endpoint=True)
    plt.plot(np.array([0,2]), np.array([0,4]), 'o', xnew,g(xnew))
    plt.show()
    print(new)
