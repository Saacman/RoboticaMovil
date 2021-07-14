"""
Saúl Isaac Sánchez Flores

Actividad 7: Mapas de Rejilla
Modifique el programa makeoccgrid.py del repositorio, para intentar que la creación del mapa sea:
(a) más rápida y/o
(b) más eficiente, es decir, que se cubra mejor el mapa o no queden huecos al avanzar el robot.
Use como base el escenario obstacles.ttt del repositorio, puede modificar esa escena para cambiar
el alcance o cobertura de los sensores. En general, se pide que use su creatividad para intentar
mejorar el programa base.


Occupancy grid creation using a Pioneer pd3x with ultrasonic sensors.
Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""
import random as rnd
import sys
import numpy as np
import time
import math as m
import random
import matplotlib.pyplot as plt
import os
import sim as vrep # access all the VREP elements
from skimage.draw import line
#<-----------------------------------Control----------------------------------------->
# Controller gains (linear and heading)

#r = 0.5*0.195
#L = 0.311
Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2

#<----------------------------------Functions---------------------------------------->
def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1/2*(x**2+y**2)
    return R

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def continuosControl(currPos, currRot, goal):
    """
    Provide control for the piooner 3dx given a goal
    """
    xd = goal[0]
    yd = goal[1]
    errp = m.sqrt((xd-currPos[0])**2 + (yd-currPos[1])**2)
    angd = m.atan2(yd-currPos[1], xd-currPos[0])
    errh = angdiff(currRot[2], angd)
    v = Kv*errp
    omega = Kh*errh
    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    return errp, ul, ur

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
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

errp = 10
goal = (rnd.randint(-6,6), rnd.randint(-6,6))
epochs = 10000
if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((100,100))
    tocc = np.zeros((100,100))
t = time.time()

initt = t
niter = 0
while time.time()-t < 300:
    epochs = epochs -1
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)

    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)

    uread = []
    ustate = []
    upt = []
    for i in range(0,16):
       err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
       ret, objpos = vrep.simxGetObjectPosition(clientID, detectedObj, -1, vrep.simx_opmode_blocking)
       uread.append(np.linalg.norm(point))
       upt.append(point)
       ustate.append(state)
       ret, srot = vrep.simxGetObjectQuaternion(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       ret, spos = vrep.simxGetObjectPosition(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       R = q2R(srot[0], srot[1], srot[2], srot[3])
       spos = np.array(spos).reshape((3,1))
       if i % 2 != 0:
           continue
       if state == True:

           opos = np.array(point).reshape((3,1))

           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100

           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0
           tocc[yo-1, xo-1] = 1

       else:
           opos = np.array([0,0,1]).reshape((3,1))

           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0
    ul = 2.0
    ur = 2.0
    lgains = np.linspace(0,-1,8)
    rgains = np.linspace(-1,0,8)
    for i in range(8):
        if ustate[i]:
            ul = ul + lgains[i]*(1.0 - uread[i])
            ur = ur + rgains[i]*(1.0 - uread[i])
    # Set a random goal in the map to follow every n cycles or if we reach a goal
    if epochs <= 0 or errp < 0.2:
        goal = (rnd.randint(-6,6), rnd.randint(-6,6))
        epochs = 10000
    #print('lvel {}   rvel {}'.format(ul, ur))
    # Check if the braitenberg algorithm is critical, if not try to follow a random point
    if abs(ul-2.0) < 0.1 and abs(ur-2.0) < 0.1: #change the critical value to decide when to ignore the algorithm
        # Call continuos control function to try to follow the point
        errp, ul, ur = continuosControl(carpos, carrot, goal)


    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)

    niter = niter + 1

#print(lgains)
#print(rgains)
finalt = time.time()
#print('Avg time per iteration ', (finalt-initt)/niter)

plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0.0, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0.0, vrep.simx_opmode_streaming)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)