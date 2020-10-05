#Saúl Isaac Sánchez Flores
# Septiembre 2020
# Instrucciones: 

# Modifique el programa vreptest.py del repositorio para causar que el robot Pioneer 3dx recorra una
#  ruta con forma de cuadrado de 2 metros por lado. Esto es, haga que el robot
# Avance 2 metros.
# Gire 90 grados.
# Realizar estos dos movimientos 4 veces causará que el robot trace el cuadrado y quede 
# aproximadamente con la misma orientación que inició.

# Cuando envíe su actividad, además del archivo de Python .py envíe el escenario de CoppeliaSim 
# que utilizó (extensión .ttt).
"""
Demonstration of a Python-CoppeliaSim connection using a Pioneer 3dx simulation

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""
import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements

def goNmeters(distance, seconds): #Could add ID and handlers as arguments
    # Assuming rad/s and r = 97.5mm
    vel = (distance * 1000) / seconds
    ang_vel = vel / 97.5 #rad/s
    t = time.time()
    while (time.time()-t) < seconds:
        err = vrep.simxSetJointTargetVelocity(clientID, motorL, ang_vel, vrep.simx_opmode_streaming)
        if err < 0:
            print("ERROR L")
        err = vrep.simxSetJointTargetVelocity(clientID, motorR, ang_vel, vrep.simx_opmode_streaming)
        if err < 0:
            print("ERROR R")
        time.sleep(0.1)

def turnRight(radians, seconds):
    s = (radians / 4) * 380 #mm
    vel = s / seconds
    ang_vel = vel / 97.5 
    t = time.time()
    while (time.time()-t) < seconds:

        err = vrep.simxSetJointTargetVelocity(clientID, motorL, ang_vel, vrep.simx_opmode_streaming)
        if err < 0:
            print("ERROR R")
        err = vrep.simxSetJointTargetVelocity(clientID, motorR, -ang_vel, vrep.simx_opmode_streaming)
        if err < 0:
            print("ERROR R")
        time.sleep(0.1)

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

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

goNmeters(2, 10)
turnRight(m.pi / 2, 3)
goNmeters(2, 10)
turnRight(m.pi / 2, 3)
goNmeters(2, 10)
turnRight(m.pi / 2, 3)
goNmeters(2, 10)
turnRight(m.pi / 2, 3)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
