"""
Actividad 5: Sensores y simulación

Modifique el programa vreptest.py del repositorio para causar que el robot Pioneer 3dx determine la 
ubicación absoluta de un obstáculo en el escenario, usando sus sensores ultrasónicos.

1. Haga un escenario en CoppeliaSim que solamente contenga al robot Pioneer p3dx y un obstáculo 
(un cilindro delgado como en el video). Coloque el cilindro en el escenario de manera que al menos
 un sensor ultrasónico del robot lo detecte.
2. Como en el video, detecte la posición del obstáculo con respecto al sensor y guárdela.
3. Usando el simulador y apoyándose con los métodos simxGetObjectPosition y simxGetObjectOrientation 
(o simxGetObjectQuaternion), encuentre la posición del obstáculo con respecto al mundo o marco de 
referencia global. Use el siguiente procedimiento: lea la posición del obstáculo con respecto al 
sensor que lo está detectando con simxReadProximitySensor, use simxGetObjectPosition y 
imxGetObjectQuaternion para encontrar la pose del sensor con respecto al marco de referencia global,
 y realice las transformaciones y operaciones necesarias para encontrar la pose del obstáculo con 
 respecto al marco de referencia global.
4. Use simxGetObjectPosition con el handle de su obstáculo para obtener su posición exacta en el marco de referencia global, y compárela con la posición que obtuvo en el punto 3.
"""
"""
Demonstration of a Python-CoppeliaSim connection using a Pioneer 3dx simulation

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""
import numpy as np
import time
import math as m
import sys
import frameplot as fp
def transformB2A(euler, trans, pos_b):
    alpha = euler[0]
    beta = euler[1]
    gamma = euler[2]
    AB_R = np.dot(fp.rotx(alpha), np.dot(fp.roty(beta), fp.rotz(gamma)))
    rotated = np.dot(AB_R, pos_b.reshape(3, 1))
    return rotated + np.array(trans).reshape(3,1)

import sim as vrep # access all the VREP elements
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)
sensord = {}
# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)
    sensord[i] = []

t = time.time()
indexes = []
while (time.time() - t) < 2:
    # Poll the sensors
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        if state:
            # This only works for static sensors
            sensord[i].append(point)
            if i not in indexes:
                indexes.append(i)


err, cylinder = vrep.simxGetObjectHandle(clientID, 'Cylinder', vrep.simx_opmode_blocking)
for i in indexes:
    cyl_pos = np.array(sensord[i])
    # Use the average of the readings of each sensor
    cyl_pos = np.average(cyl_pos, axis=0)
    print(f"Sensor {i} detection at: {cyl_pos}\n")
    sensor_pos = vrep.simxGetObjectPosition(clientID, usensor[i], -1, vrep.simx_opmode_oneshot_wait)
    sensor_orien = vrep.simxGetObjectOrientation(clientID, usensor[i], -1, vrep.simx_opmode_oneshot_wait)
    global_pos= transformB2A(sensor_orien[1], sensor_pos[1], cyl_pos)
    handle_pos = vrep.simxGetObjectPosition(clientID, cylinder, -1, vrep.simx_opmode_oneshot_wait)
    print(f"Calculated Position:\n{global_pos}")
    print(f"Simulation Position: {handle_pos}\n\n")



vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
