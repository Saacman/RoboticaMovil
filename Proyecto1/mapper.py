"""
Proyecto 2: Mapas de ocupación
"""
import matplotlib.pyplot as plt
import numpy as np
import time
import math as m
import sys
import sim as vrep  # access all the VREP elements
from mapping import transformB2A, GridMap
from skimage.draw import line
import os 
# <---------------------------------Initialization--------------------------------------->
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', -1, True, True,
                          5000, 5)  # start a connection
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Not connected to remote API server')
    sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1, 17):
    err, s = vrep.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(
        clientID, usensor[i], vrep.simx_opmode_streaming)


# Initialize the map
err, pos0 = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
pos0 = np.array(pos0)
csize = 0.1  # 10 cm

# Don't move the robot (TODO: save the initial position in the grid object, 
# translate the coords in setPoint)

if os.path.exists('map.png'):
    print("Loading map...")
    mp = GridMap.loadImg("map.png")
else:
    mp = GridMap()
    
t = time.time()
while time.time()-t < 60:
    err, carpos = vrep.simxGetObjectPosition(
        clientID, robot, -1, vrep.simx_opmode_oneshot_wait)
    carpos = np.array(carpos)
    gpos = ((carpos - pos0) / csize).astype(int)
    #mp.setPoint(gpos, 0)
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
            # Añadir el punto detectado
            mp.setPoint(grid_pos, 1)
            rr, cc = line(gpos[1], gpos[0], grid_pos[1], grid_pos[0])
            rr = rr + mp.coffset[1]
            cc = cc + mp.coffset[0]
            # Añadir los puntos entre la posición y el detectado
            mp.grid[rr[:-2],cc[:-2]] = 0
            
        else:
            global_pos = transformB2A(sensor_orien[1], sensor_pos[1], np.array([0, 0, 1]))
            grid_pos = ((global_pos - pos0) / csize).astype(int) 
            mp.setPoint(grid_pos, 0)
            rr, cc = line(gpos[1], gpos[0], grid_pos[1], grid_pos[0])
            rr = rr + mp.coffset[1]
            cc = cc + mp.coffset[0]
            mp.grid[rr[:-2],cc[:-2]] = 0
            

# The End
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
final = mp.getGrid()
mp.saveImg("map.png")
plt.imshow(final)
plt.show()

