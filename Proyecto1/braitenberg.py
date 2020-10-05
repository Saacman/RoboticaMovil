import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements

#<---------------------------------Initialization--------------------------------------->
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
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(
        clientID, usensor[i], vrep.simx_opmode_streaming)

#<---------------------------------Braitenberg--------------------------------------->
noDetectDist = 0.5
maxDist = 0.2
detectW = np.zeros(16)
braitenbergL = np.array([-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
braitenbergR = np.array([-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
v0=4
t = time.time()

while True:
    # Poll the sensors
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        distance = np.linalg.norm(point)
        # if a detection occurs
        if state and (distance < noDetectDist): # don't care about distant objects
            distance = max(distance, maxDist) 
            detectW[i] = 1 - ((distance - maxDist) / (noDetectDist - maxDist)) # Normalize the weight
        else:
            detectW[i] = 0
        
    
    vLeft = v0 + np.sum(braitenbergL * detectW)
    vRight = v0 + np.sum(braitenbergR * detectW)

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, vLeft, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, vRight, vrep.simx_opmode_streaming)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)