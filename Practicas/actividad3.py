"""
En esta ocasión, para practicar el movimiento de marcos de referencia en 3D,
 escribirán un programa corto en Python, usando Numpy, Matplotlib y Scipy. 
 Pueden basarse en los ejemplos del video, que están en el repositorio de Github del curso.

Instrucciones: Escriba un programa en Python, dentro de un solo archivo .py 
que será su entrega de la actividad. Modifique el programa llamado "rotveldemo.py"
 del repositorio del curso, para que la velocidad lineal o traslacional cambie de
  acuerdo con una función del tiempo

x(t) = t^2
y(t) = t
z(t) = 0

La velocidad rotacional déjela tal como está en el código. Igual que el programa
original, el suyo debe graficar la evolución del marco de referencia móvil a través del tiempo.

Tal como antes, agregue los comentarios que considere necesarios para entender su código. """
"""
Rotational and translational velocities

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

"""
Rotational and translational velocities

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import frameplot as fp

# Start by defining the infinitesimal increment for rotational and lineal velocity

# Now the displacement is a time function

# Constant rotation over the Z axis at pi/2 rad per second
deltav = np.matrix([[0,0,np.pi/2]]).T

# Time step in seconds
dt = 0.1 # Incremented to appreciate the movement better.

tvec = []
rmat = []
t = np.matrix([[0,0,0]]).T
R = np.eye(3)
path = np.zeros((2,50))

# Moving the frame during 50 time steps
for i in range(50):
    # We need the accumulated time, which is dt * i
    elaps_t = dt * i
    # t = t + dt*np.matrix([[elaps_t * elaps_t, elaps_t, 0]]).T
    # Usando las derivadas:
    t = t + dt*np.matrix([[2 * elaps_t, 1, 0]]).T
    R = (dt*fp.skew(deltav)+np.eye(3))*R
    tvec.append(t)
    rmat.append(R)
    path[0,i] = t[0,0]
    path[1,i] = t[1,0]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)

for i in range(0, 50, 10):
    fp.frame(ax, tvec[i], rmat[i], flabel='t='+str(dt*i), fcolor=(0,0,i/50))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

fig2 = plt.figure()
ax = fig2.gca()
ax.plot(path[0,:], path[1,:])
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.show()
print(t)
print(R)
