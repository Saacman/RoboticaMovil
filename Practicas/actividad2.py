# Python3
# Robótica Móvil 
# Saúl Isaac Sánchez Flores

# Actividad 2
# En esta ocasión, para practicar la generación de trayectorias, 
# escribirán un programa corto en Python, usando Numpy, Matplotlib y Scipy.
# Pueden basarse en los ejemplos del video, que están en el repositorio de 
# Github del curso.

# Instrucciones: Escriba un programa en Python, dentro de un solo archivo .py
#  que será su entrega de la actividad. Este programa deberá generar una lista 
#  de 10 puntos aleatorios en el plano XY, y debe a continuación calcular una 
#  trayectoria suave que pase por todos los puntos generados (puede usar b-splines).
#  Haga que la trayectoria consista de 200 instantes de tiempo,
#  con sus correspondientes coordenadas X y Y.
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as spi


totaltime = 10
# Puntos a visitar
x_p = np.sort(np.random.rand(15)) 
y_p = np.random.rand(15)
t_p = totaltime * x_p / x_p[-1]

# Instantes de tiempo
t_values = np.linspace(0, totaltime, 200)
# Crear splines
xc = spi.splrep(t_p, x_p, s=0)
yc = spi.splrep(t_p, y_p, s=0)
# Evaluar
x_values = spi.splev(t_values, xc, der=0)
y_values = spi.splev(t_values, yc, der=0)

plt.plot(x_values, y_values)
plt.plot(x_p, y_p, '.r')
plt.suptitle('Trayectoria Generada')
plt.show()